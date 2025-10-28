FROM ubuntu:24.04

# Install required packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    libboost-all-dev \
    libosmium2-dev \
    libprotozero-dev \
    libexpat1-dev \
    zlib1g-dev \
    libbz2-dev \
    libutfcpp-dev \
    pkg-config \
    libgdal-dev \
    osmium-tool

# Set the working directory
WORKDIR /usr/src/app

# Copy the whole project into the container
COPY . .

# Create build directory
RUN rm -rf build && mkdir build && cd build && cmake .. && make

CMD ["/bin/bash"]

