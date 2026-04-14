#include <httplib.h>
#include <proj.h>
#include <tiffio.h>

#include <cstring>
#include <iostream>
#include <px4_rerun/terrain_fetcher.hpp>
#include <sstream>
#include <string>
#include <vector>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define TINYGLTF_IMPLEMENTATION
#define TINYGLTF_NO_STB_IMAGE
#define TINYGLTF_NO_STB_IMAGE_WRITE
#include <tiny_gltf.h>

namespace terrain_fetcher {
namespace {

// ── Structs ──────────────────────────────────────────────────────────────────

struct ElevationData {
  std::vector<float> elevations;  // row-major (rows x cols)
  int rows = 0;
  int cols = 0;
  double lon_min, lat_min, lon_max, lat_max;
};

struct ImageData {
  std::vector<uint8_t> rgb;  // row-major (H x W x 3)
  int width = 0;
  int height = 0;
};

struct Mesh {
  std::vector<float> positions;   // flat (N*3): east, north, up
  std::vector<float> uvs;         // flat (N*2)
  std::vector<uint32_t> indices;  // flat (M*3)
};

// ── HTTP helpers ─────────────────────────────────────────────────────────────

constexpr int TIMEOUT_SEC = 60;

std::string https_get(const std::string& host, const std::string& path) {
  httplib::SSLClient cli(host);
  cli.set_read_timeout(TIMEOUT_SEC);
  cli.set_connection_timeout(TIMEOUT_SEC);

  auto res = cli.Get(path);
  if (!res) {
    std::cerr << "HTTP request failed: " << host << path << "\n";
    return {};
  }
  if (res->status != 200) {
    std::cerr << "HTTP " << res->status << " from " << host << path << "\n";
    return {};
  }

  auto ct = res->get_header_value("Content-Type");
  if (ct.find("application/json") != std::string::npos) {
    std::cerr << "API error (JSON response): " << res->body.substr(0, 200) << "\n";
    return {};
  }

  if (res->body.size() < 1024) {
    std::cerr << "Response too small (" << res->body.size()
              << " bytes) — area likely outside US coverage\n";
    return {};
  }

  return res->body;
}

// ── TIFF in-memory read ──────────────────────────────────────────────────────

struct TiffMemBuf {
  const uint8_t* data;
  tsize_t size;
  toff_t offset;
};

static tsize_t tiff_read(thandle_t h, tdata_t buf, tsize_t n) {
  auto* m = static_cast<TiffMemBuf*>(h);
  tsize_t avail = m->size - m->offset;
  tsize_t to_read = n < avail ? n : avail;
  std::memcpy(buf, m->data + m->offset, to_read);
  m->offset += to_read;
  return to_read;
}
static tsize_t tiff_write(thandle_t, tdata_t, tsize_t) { return 0; }
static toff_t tiff_seek(thandle_t h, toff_t off, int whence) {
  auto* m = static_cast<TiffMemBuf*>(h);
  switch (whence) {
    case SEEK_SET:
      m->offset = off;
      break;
    case SEEK_CUR:
      m->offset += off;
      break;
    case SEEK_END:
      m->offset = m->size + off;
      break;
  }
  return m->offset;
}
static int tiff_close(thandle_t) { return 0; }
static toff_t tiff_size(thandle_t h) { return static_cast<TiffMemBuf*>(h)->size; }

// ── Fetch elevation ──────────────────────────────────────────────────────────

ElevationData fetch_elevation(double lat_min, double lat_max, double lon_min, double lon_max) {
  std::ostringstream path;
  path << "/arcgis/rest/services/3DEPElevation/ImageServer/exportImage?"
       << "bbox=" << lon_min << "," << lat_min << "," << lon_max << "," << lat_max
       << "&bboxSR=4326&imageSR=4326&format=tiff&pixelType=F32"
       << "&noDataInterpretation=esriNoDataMatchAny"
       << "&interpolation=RSP_BilinearInterpolation&f=image";

  auto body = https_get("elevation.nationalmap.gov", path.str());
  if (body.empty()) return {};

  // Decode float32 GeoTIFF from memory
  TiffMemBuf membuf{reinterpret_cast<const uint8_t*>(body.data()),
                    static_cast<tsize_t>(body.size()), 0};
  TIFF* tif = TIFFClientOpen("mem", "rm", &membuf, tiff_read, tiff_write, tiff_seek, tiff_close,
                             tiff_size, nullptr, nullptr);
  if (!tif) {
    std::cerr << "Failed to open TIFF from memory\n";
    return {};
  }

  uint32_t w = 0, h = 0;
  TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &w);
  TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &h);

  if (w > 8000 || h > 8000) {
    std::cerr << "Elevation grid too large: " << w << "x" << h << "\n";
    TIFFClose(tif);
    return {};
  }

  ElevationData elev;
  elev.cols = static_cast<int>(w);
  elev.rows = static_cast<int>(h);
  elev.lon_min = lon_min;
  elev.lat_min = lat_min;
  elev.lon_max = lon_max;
  elev.lat_max = lat_max;
  elev.elevations.resize(elev.rows * elev.cols);

  if (TIFFIsTiled(tif)) {
    uint32_t tw = 0, th = 0;
    TIFFGetField(tif, TIFFTAG_TILEWIDTH, &tw);
    TIFFGetField(tif, TIFFTAG_TILELENGTH, &th);
    std::vector<float> tile_buf(tw * th);

    for (uint32_t y = 0; y < h; y += th) {
      for (uint32_t x = 0; x < w; x += tw) {
        TIFFReadTile(tif, tile_buf.data(), x, y, 0, 0);
        uint32_t copy_h = std::min(th, h - y);
        uint32_t copy_w = std::min(tw, w - x);
        for (uint32_t r = 0; r < copy_h; ++r) {
          std::memcpy(&elev.elevations[(y + r) * w + x], &tile_buf[r * tw], copy_w * sizeof(float));
        }
      }
    }
  } else {
    for (int row = 0; row < elev.rows; ++row) {
      TIFFReadScanline(tif, &elev.elevations[row * elev.cols], row);
    }
  }
  TIFFClose(tif);

  std::cerr << "Elevation: " << elev.cols << "x" << elev.rows << " px\n";
  return elev;
}

// ── Fetch imagery ────────────────────────────────────────────────────────────

ImageData fetch_imagery(double lat_min, double lat_max, double lon_min, double lon_max) {
  std::ostringstream path;
  path << "/arcgis/rest/services/USGSNAIPImagery/ImageServer/exportImage?"
       << "bbox=" << lon_min << "," << lat_min << "," << lon_max << "," << lat_max
       << "&bboxSR=4326&imageSR=4326&format=png&f=image";

  auto body = https_get("imagery.nationalmap.gov", path.str());
  if (body.empty()) return {};

  int w, h, channels;
  auto* pixels =
      stbi_load_from_memory(reinterpret_cast<const uint8_t*>(body.data()),
                            static_cast<int>(body.size()), &w, &h, &channels, 3);  // force RGB

  if (!pixels) {
    std::cerr << "Failed to decode imagery PNG\n";
    return {};
  }

  ImageData img;
  img.width = w;
  img.height = h;
  img.rgb.assign(pixels, pixels + w * h * 3);
  stbi_image_free(pixels);

  std::cerr << "Imagery: " << w << "x" << h << " px\n";
  return img;
}

// ── Coordinate conversion ────────────────────────────────────────────────────

std::vector<float> convert_to_local_enu(const ElevationData& elev, double ref_lat, double ref_lon,
                                        double ref_alt) {
  int rows = elev.rows, cols = elev.cols;
  int n = rows * cols;

  // Generate lon/lat grid
  std::vector<double> lons(n), lats(n);
  for (int r = 0; r < rows; ++r) {
    double lat = elev.lat_max - (elev.lat_max - elev.lat_min) * r / (rows - 1);
    for (int c = 0; c < cols; ++c) {
      double lon = elev.lon_min + (elev.lon_max - elev.lon_min) * c / (cols - 1);
      int idx = r * cols + c;
      lats[idx] = lat;
      lons[idx] = lon;
    }
  }

  PJ_CONTEXT* ctx = proj_context_create();

  // Vertical: USGS 3DEP returns NAVD88 (geoid). PX4's vehicle_local_position.ref_alt
  // is AMSL per spec (also geoid-referenced, typically EGM96/EGM2008 from the GPS).
  // NAVD88 and EGM96/EGM2008 differ by <1m in CONUS, so we treat them as directly
  // comparable and skip the NAVD88→WGS84 ellipsoid conversion (which would introduce
  // a ~20m offset since the geoid sits below the ellipsoid).

  // Horizontal: WGS84 → local ENU (azimuthal equidistant)
  std::string aeqd_def = "+proj=aeqd +lat_0=" + std::to_string(ref_lat) +
                         " +lon_0=" + std::to_string(ref_lon) + " +datum=WGS84";
  PJ* aeqd = proj_create_crs_to_crs(ctx, "EPSG:4326", aeqd_def.c_str(), nullptr);
  if (!aeqd) {
    std::cerr << "PROJ: failed to create aeqd transformer\n";
    proj_context_destroy(ctx);
    return {};
  }

  PJ* aeqd_norm = proj_normalize_for_visualization(ctx, aeqd);
  proj_destroy(aeqd);
  if (!aeqd_norm) {
    std::cerr << "PROJ: failed to normalize aeqd transformer\n";
    proj_context_destroy(ctx);
    return {};
  }

  std::vector<float> vertices(n * 3);
  for (int i = 0; i < n; ++i) {
    PJ_COORD in = proj_coord(lons[i], lats[i], 0, 0);
    PJ_COORD out = proj_trans(aeqd_norm, PJ_FWD, in);
    vertices[i * 3 + 0] = static_cast<float>(out.xy.x);                      // east
    vertices[i * 3 + 1] = static_cast<float>(out.xy.y);                      // north
    vertices[i * 3 + 2] = static_cast<float>(elev.elevations[i] - ref_alt);  // up
  }

  proj_destroy(aeqd_norm);
  proj_context_destroy(ctx);

  return vertices;
}

// ── Mesh generation ──────────────────────────────────────────────────────────

Mesh build_mesh(int rows, int cols, std::vector<float> vertices) {
  Mesh mesh;
  mesh.positions = std::move(vertices);

  // Triangulate grid: 2 triangles per cell
  int cells = (rows - 1) * (cols - 1);
  mesh.indices.reserve(cells * 6);
  for (int r = 0; r < rows - 1; ++r) {
    for (int c = 0; c < cols - 1; ++c) {
      uint32_t tl = r * cols + c;
      uint32_t tr = tl + 1;
      uint32_t bl = (r + 1) * cols + c;
      uint32_t br = bl + 1;
      mesh.indices.push_back(tl);
      mesh.indices.push_back(bl);
      mesh.indices.push_back(tr);
      mesh.indices.push_back(tr);
      mesh.indices.push_back(bl);
      mesh.indices.push_back(br);
    }
  }

  // UV coordinates: u = col/(cols-1), v = 1 - row/(rows-1)
  int n = rows * cols;
  mesh.uvs.resize(n * 2);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      int idx = r * cols + c;
      mesh.uvs[idx * 2 + 0] = static_cast<float>(c) / (cols - 1);
      mesh.uvs[idx * 2 + 1] = static_cast<float>(r) / (rows - 1);
    }
  }

  return mesh;
}

// ── PNG encode helper ────────────────────────────────────────────────────────

static void png_write_callback(void* context, void* data, int size) {
  auto* buf = static_cast<std::vector<uint8_t>*>(context);
  auto* bytes = static_cast<uint8_t*>(data);
  buf->insert(buf->end(), bytes, bytes + size);
}

// ── GLB export ───────────────────────────────────────────────────────────────

std::vector<uint8_t> export_glb(const Mesh& mesh, const ImageData& texture) {
  // Re-encode texture as PNG
  std::vector<uint8_t> png_data;
  stbi_write_png_to_func(png_write_callback, &png_data, texture.width, texture.height, 3,
                         texture.rgb.data(), texture.width * 3);

  // Build binary buffer: positions | uvs | indices | png
  int n_verts = static_cast<int>(mesh.positions.size() / 3);
  size_t pos_bytes = mesh.positions.size() * sizeof(float);
  size_t uv_bytes = mesh.uvs.size() * sizeof(float);
  size_t idx_bytes = mesh.indices.size() * sizeof(uint32_t);
  size_t png_bytes = png_data.size();

  size_t pos_offset = 0;
  size_t uv_offset = pos_bytes;
  size_t idx_offset = uv_offset + uv_bytes;
  size_t png_offset = idx_offset + idx_bytes;
  size_t total = png_offset + png_bytes;

  std::vector<uint8_t> buffer(total);
  std::memcpy(buffer.data() + pos_offset, mesh.positions.data(), pos_bytes);
  std::memcpy(buffer.data() + uv_offset, mesh.uvs.data(), uv_bytes);
  std::memcpy(buffer.data() + idx_offset, mesh.indices.data(), idx_bytes);
  std::memcpy(buffer.data() + png_offset, png_data.data(), png_bytes);

  // Compute position bounding box for accessor
  float min_pos[3] = {1e30f, 1e30f, 1e30f};
  float max_pos[3] = {-1e30f, -1e30f, -1e30f};
  for (int i = 0; i < n_verts; ++i) {
    for (int j = 0; j < 3; ++j) {
      float v = mesh.positions[i * 3 + j];
      if (v < min_pos[j]) min_pos[j] = v;
      if (v > max_pos[j]) max_pos[j] = v;
    }
  }

  tinygltf::Model model;
  model.asset.version = "2.0";
  model.asset.generator = "terrain-fetcher-cpp";

  // Buffer
  tinygltf::Buffer buf;
  buf.data = std::move(buffer);
  model.buffers.push_back(std::move(buf));

  // Buffer views: 0=positions, 1=uvs, 2=indices, 3=png
  auto add_view = [&](size_t offset, size_t length, int target) {
    tinygltf::BufferView bv;
    bv.buffer = 0;
    bv.byteOffset = offset;
    bv.byteLength = length;
    bv.target = target;
    model.bufferViews.push_back(std::move(bv));
    return static_cast<int>(model.bufferViews.size() - 1);
  };

  int bv_pos = add_view(pos_offset, pos_bytes, TINYGLTF_TARGET_ARRAY_BUFFER);
  int bv_uv = add_view(uv_offset, uv_bytes, TINYGLTF_TARGET_ARRAY_BUFFER);
  int bv_idx = add_view(idx_offset, idx_bytes, TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER);
  int bv_img = add_view(png_offset, png_bytes, 0);

  // Accessors: 0=positions, 1=uvs, 2=indices
  {
    tinygltf::Accessor acc;
    acc.bufferView = bv_pos;
    acc.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
    acc.count = n_verts;
    acc.type = TINYGLTF_TYPE_VEC3;
    acc.minValues = {min_pos[0], min_pos[1], min_pos[2]};
    acc.maxValues = {max_pos[0], max_pos[1], max_pos[2]};
    model.accessors.push_back(std::move(acc));
  }
  {
    tinygltf::Accessor acc;
    acc.bufferView = bv_uv;
    acc.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
    acc.count = n_verts;
    acc.type = TINYGLTF_TYPE_VEC2;
    model.accessors.push_back(std::move(acc));
  }
  {
    tinygltf::Accessor acc;
    acc.bufferView = bv_idx;
    acc.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
    acc.count = static_cast<int>(mesh.indices.size());
    acc.type = TINYGLTF_TYPE_SCALAR;
    model.accessors.push_back(std::move(acc));
  }

  // Image (embedded PNG)
  tinygltf::Image img;
  img.bufferView = bv_img;
  img.mimeType = "image/png";
  model.images.push_back(std::move(img));

  // Sampler
  tinygltf::Sampler sampler;
  sampler.magFilter = TINYGLTF_TEXTURE_FILTER_LINEAR;
  sampler.minFilter = TINYGLTF_TEXTURE_FILTER_LINEAR;
  model.samplers.push_back(std::move(sampler));

  // Texture
  tinygltf::Texture tex;
  tex.source = 0;
  tex.sampler = 0;
  model.textures.push_back(std::move(tex));

  // Material (PBR)
  tinygltf::Material mat;
  mat.pbrMetallicRoughness.baseColorTexture.index = 0;
  mat.pbrMetallicRoughness.metallicFactor = 0.0;
  mat.pbrMetallicRoughness.roughnessFactor = 1.0;
  model.materials.push_back(std::move(mat));

  // Mesh primitive
  tinygltf::Primitive prim;
  prim.attributes["POSITION"] = 0;
  prim.attributes["TEXCOORD_0"] = 1;
  prim.indices = 2;
  prim.material = 0;
  prim.mode = TINYGLTF_MODE_TRIANGLES;

  tinygltf::Mesh gltf_mesh;
  gltf_mesh.primitives.push_back(std::move(prim));
  model.meshes.push_back(std::move(gltf_mesh));

  // Node
  tinygltf::Node node;
  node.mesh = 0;
  model.nodes.push_back(std::move(node));

  // Scene
  tinygltf::Scene scene;
  scene.nodes.push_back(0);
  model.scenes.push_back(std::move(scene));
  model.defaultScene = 0;

  // Write GLB to memory
  tinygltf::TinyGLTF writer;
  std::ostringstream oss;
  writer.WriteGltfSceneToStream(&model, oss, false, true);  // prettyPrint=false, binary=true
  std::string glb_str = oss.str();

  return {glb_str.begin(), glb_str.end()};
}

}  // anonymous namespace

// ── Public API ───────────────────────────────────────────────────────────────

TerrainData fetch_terrain(double ref_lat, double ref_lon, double ref_alt, double lat_min,
                          double lat_max, double lon_min, double lon_max) {
  auto elev = fetch_elevation(lat_min, lat_max, lon_min, lon_max);
  if (elev.elevations.empty()) return {};

  auto imagery = fetch_imagery(lat_min, lat_max, lon_min, lon_max);
  if (imagery.rgb.empty()) return {};

  auto vertices = convert_to_local_enu(elev, ref_lat, ref_lon, ref_alt);
  if (vertices.empty()) return {};

  auto mesh = build_mesh(elev.rows, elev.cols, std::move(vertices));
  auto glb = export_glb(mesh, imagery);

  std::cerr << "Terrain GLB: " << glb.size() << " bytes\n";
  return {std::move(glb)};
}

}  // namespace terrain_fetcher
