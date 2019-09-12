Installation
============

```
git clone <...>
cd cpp-utils
mkdir -p build/Release # use "Debug" if you want to build in debug
cd build/Release
cmake ../..
make
sudo make install
sudo ldconfig
```

Uninstall
=========

```
cd build/Release
sudo make uninstall
```
