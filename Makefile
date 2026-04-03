BUILD_DIR := build

.PHONY: all loader clean

all:
	cmake -S . -B $(BUILD_DIR)
	make -C $(BUILD_DIR) -j$$(nproc)

loader:
	cmake -S . -B $(BUILD_DIR) -DPX4_RERUN_LOADER=ON
	make -C $(BUILD_DIR) -j$$(nproc)

clean:
	rm -rf $(BUILD_DIR)
