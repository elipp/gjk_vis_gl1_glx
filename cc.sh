#!/bin/bash
CAR_DIR=/home/elias/programming/c_c++/car_win32/car
LIBS="-lGLEW -lGL -lX11"

set -e 

rebuild_depobjs=0
set -- $(getopt r "$@")
while [ $# -gt 0 ]
do
	case "$1" in
		(-r) rebuild_depobjs=1; shift;;
		(--) shift; break;;
		(-*) echo "$0: warning: unrecognized option $1" 1>&2; exit 1;;
		(*) break;;
	esac
	shift
done

CC="clang++ -O2"
if [ $rebuild_depobjs -eq 1 ]; then
	$CC -c -Wall $LIBS -DLINALG_STANDALONE -I$CAR_DIR/include $CAR_DIR/src/lin_alg.cpp -o objs/lin_alg.o
fi

BIN="gjk_vis convex_hull_test"
DEPOBJS=objs/lin_alg.o

rm $BIN ||
$CC -Wall -std=c++11 $LIBS -DLINALG_STANDALONE $DEPOBJS OBB.cpp -I$CAR_DIR/include main.cpp -o gjk_vis
$CC -Wall -fopenmp -DLINALG_STANDALONE $DEPOBJS -I$CAR_DIR/include convex_hull.cpp -o convex_hull_test
