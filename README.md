# HS3MDP

POMCP enhancement for Hidden Semi-Markovian Mode MDP.

HS3MDP is based on [POMCP 1.0](http://www0.cs.ucl.ac.uk/staff/D.Silver/web/Applications.html) used in the NIPS 2010 paper
_Online Monte-Carlo Planning in Large POMDPs_ by David Silver and Joel Veness.

The original code can be found in this repository under the tag ```POMCP-1.0```.

## Requirements
* git
* automake, autoconf, etc.
* C++ Boost

## Installation
1. ```git clone``` this repository
2. run ```autoreconf -i```
3. run ```./configure``` (possibly with ```--enable-assert```)
4. run ```make```

You will find the executable ```pomcp``` in the ```src``` directory.

## Usage
Simply run ```pomcp --help``` to see all possible parameters.
