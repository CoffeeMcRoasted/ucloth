mkdir build
cd build
cmake .. -G"Visual Studio 15 2017 Win64"
"C:\Program Files (x86)\Microsoft Visual Studio\2017\BuildTools\MSBuild\15.0\Bin\msbuild.exe" ucloth.sln /target:ALL_BUILD