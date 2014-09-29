bin_PROGRAMS = pomcp

pomcp_SOURCES = \
battleship.cpp \
beliefstate.cpp \
coord.cpp \
experiment.cpp \
main.cpp \
mcts.cpp \
network.cpp \
node.cpp \
pocman.cpp \
rocksample.cpp \
simulator.cpp \
tag.cpp \
testsimulator.cpp \
utils.cpp \
pomdp.cpp \
hs3mdp.cpp \
sparsehs3mdp.cpp \
trafficlight.cpp \
sailboat.cpp

noinst_HEADERS = \
battleship.h \
beliefstate.h \
coord.h \
experiment.h \
grid.h \
history.h \
mcts.h \
memorypool.h \
network.h \
node.h \
pocman.h \
rocksample.h \
simulator.h \
statistic.h \
tag.h \
testsimulator.h \
utils.h \
pomdp.h \
hs3mdp.h \
sparsehs3mdp.h \
trafficlight.h \
sailboat.h

pomcp_LDFLAGS = $(BOOST_LDFLAGS)

pomcp_LDADD = \
$(BOOST_PROGRAM_OPTIONS_LIB)

pomcp_CPPFLAGS = \
$(BOOST_CPPFLAGS)

DISTCLEANFILES = *~
