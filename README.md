## Note
```bash
xmake create -l c++ -P RecastDemo
cd RecastDemo
xmake config -m debug
# build
xmake -D -y
# run
xmake run
# run debug mode
xmake run -d RecastDemo
# generate vs project and sln files
xmake project -k vsxmake
```