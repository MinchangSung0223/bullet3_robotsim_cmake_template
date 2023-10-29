# bullet3_GUI_cmake_template
CMakeLists.txt를 열고 bullet3의 directory 경로를 확인하고 수정.
```bash
set(BULLET_PHYSICS_SOURCE_DIR  "/opt/bullet3")
set(BULLET_PHYSICS_LIB  "/opt/bullet3/build_cmake/local_install/lib")
```

# local_install/lib이 없는경우
```
  git clone https://github.com/bulletphysics/bullet3.git
  cd bullet3
  sudo mv bullet3 /opt/
  cd /opt/bullet3

  ./build_cmake_pybullet_double.sh
  cd build_cmake
  sudo make install
  cmake  -DUSE_DOUBLE_PRECISION=ON -DCMAKE_DEBUG_POSTFIX="" -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release  -DCMAKE_INSTALL_PREFIX:PATH=local_install  ..
  make -j$(nproc)
  sudo make install
  cd local_install/lib
```

# build
```
  git clone https://github.com/MinchangSung0223/bullet3_GUI_cmake_template.git
  cd bullet3_GUI_cmake_template
  mkdir build && cd build &&  cmake ..
  make -j$(nproc)

```
