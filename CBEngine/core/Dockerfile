FROM python:3.6-alpine as so-image
RUN sed -i 's/dl-cdn.alpinelinux.org/mirrors.aliyun.com/g' /etc/apk/repositories
RUN apk add --no-cache\
 g++\
 cmake\
 make
WORKDIR /city-brain
COPY externals ./externals
COPY src ./src
COPY CMakeLists.txt .
COPY main.cc .
COPY setup.py .
RUN pip install --no-cache-dir \
 -i http://mirrors.aliyun.com/pypi/simple --trusted-host mirrors.aliyun.com \
 pybind11
RUN pip install .

COPY data ./data
COPY simulator.cfg .
COPY demo.py .
ENTRYPOINT ["/sbin/init"]