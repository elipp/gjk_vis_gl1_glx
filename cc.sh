#!/bin/bash
CAR_DIR=/home/elias/programming/c_c++/car_win32/car
LIBS="-lGLEW -lGL -lX11"

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

if [ $rebuild_depobjs -gt 0 ]; then
	mkdir -p objs
	g++ -c -Wall -DLINALG_STANDALONE -I$CAR_DIR/include $CAR_DIR/src/lin_alg.cpp -o objs/lin_alg.o
	g++ -c -Wall -DLINALG_STANDALONE -I$CAR_DIR/include OBB.cpp -o objs/OBB.o
fi

DEPOBJS="objs/lin_alg.o"

rm gjk_vis 
g++ -Wall -std=c++11 $LIBS -DLINALG_STANDALONE $DEPOBJS OBB.cpp -I$CAR_DIR/include main.cpp -o gjk_vis
