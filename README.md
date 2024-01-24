# l431_demo

```bash
cd bld
cmake -G "Unix Makefiles" -H. -Bbuild -DCMAKE_TOOLCHAIN_FILE=toolchains.cmake
cmake --build build -t all -- -j${nproc}

cd app
cmake -G "Unix Makefiles" -H. -Bbuild -DCMAKE_TOOLCHAIN_FILE=toolchains.cmake
cmake --build build -t all -- -j${nproc}
```
