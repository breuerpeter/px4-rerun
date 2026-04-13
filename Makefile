BUILD_DIR := build

.PHONY: all clean

all:
	cmake -S . -B $(BUILD_DIR)
	make -C $(BUILD_DIR) -j$$(nproc)

clean:
	rm -rf $(BUILD_DIR)
