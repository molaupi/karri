# KaRRi

This repository contains the C++17 implementation of KaRRi, a state-of-the-art dispatcher for the dynamic 
taxi sharing problem with meeting points. 
KaRRi uses engineered on-the-fly shortest path queries based on bucket contraction hierarchies (BCHs) 
to allow for fast query times with maximum flexibility. 
For more information on KaRRi's novel techniques, we refer to the related publication:

* Moritz Laupichler, and Peter Sanders. Fast Many-to-Many Routing for Dynamic Taxi Sharing with
  Meeting Points. 2024 Proceedings of the Symposium on Algorithm Engineering and Experiments (ALENEX),
  2024\. https://doi.org/10.1137/1.9781611977929.6

If you use KaRRi in your scientific publication, we ask that you reference the paper above.

## License

All files in this repository except the files in the directory `External` are licensed under the MIT
license. External libraries are licensed under their respective licenses.

This source code is based on a fork of https://github.com/vbuchhold/routing-framework.
Large parts of the project structure as well as basic data structures and shortest path algorithms
are directly taken or adapted from the original framework.
The copyright statements in each file state the respective author or authors of the file.

## Prerequisites

To build KaRRi, you need to have some tools and libraries installed. On Debian and its derivatives
(such as Ubuntu) the `apt-get` tool can be used:

```
$ sudo apt-get install build-essential
$ sudo apt-get install cmake
$ sudo apt-get install python3 python3-pip; pip3 install -r python_requirements.txt
$ sudo apt-get install libboost-all-dev
$ sudo apt-get install libproj-dev
$ sudo apt-get install zlib1g-dev
$ sudo apt-get install osmium-tool
```

Next, you need to clone the libraries in the `External` subdirectory and build the `RoutingKit` library. To do so,
type the following commands at the top-level directory of the framework:

```
$ git submodule update --init
$ make -C External/RoutingKit
```


## Constructing KaRRi Input
We provide bash scripts to generate the input data for the `Berlin-1pct`, `Berlin-10pct`,
`Ruhr-1pct`, and `Ruhr-10pct` problem instances for the KaRRi algorithm. For example, you
can generate the input data for the `Berlin-1pct` instance by typing the following commands
at the top-level directory: (Downloads multiple GiB of raw OSM data and requires at least 10 GiB of RAM.)

```
$ cd Publications/KaRRi
$ bash DownloadGermanyOSMData.sh .
$ bash FilterGermanyOSMData.sh .
$ bash PreprocessOSMData.sh . Germany Berlin BoundaryPolygons
$ bash GenerateKnownInstanceInputData.sh . Berlin-1pct pedestrian
```

To generate the input data for the other instances, simply replace `Berlin-1pct` with the instance name
(`Berlin-10pct`, `Ruhr-1pct`, `Ruhr-10pct`) and replace `Berlin` with `Ruhr` for the
Ruhr instances.


## Running KaRRi
To run KaRRi in its default configuration (using collective last stop searches, sorted buckets, and
SIMD instructions), use the provided bash script by typing the following commands at the top-level directory:

```
$ cd Publications/KaRRi
$ bash RunKaRRiDefault.sh . <instance-name> <output-dir>
```

where `<instance-name>` can be any of `Berlin-1pct`, `Berlin-10pct`, `Ruhr-1pct`,
and `Ruhr-10pct`,  and `<output-dir>` is the path to the directory where the output files
will be stored.

We provide functions for a basic evaluation of results in `Publications/KaRRi/eval.R`.
