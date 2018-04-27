TERM=
#TERM=xterm -e
PRELOAD_SCRIPTS?=source ../../../devel/reldebug/setup.bash;
LAUNCHDIR=./launch/
PACKAGE=pal_joint_trajectory_controller
CLANG_FORMAT?=clang-format-4.0

test:
	cd ../../; ${MAKE} test PACKAGE=${PACKAGE}

clean:
	cd ../../; ${MAKE} clean PACKAGE=${PACKAGE}

dox:
	cd ../../; ${MAKE} dox PACKAGE=${PACKAGE}

rebuild:
	${MAKE} test
	cd ../../; ${MAKE} PACKAGE=${PACKAGE}

format:
	${CLANG_FORMAT} -style=file -i `find ./ -iname "*.h" -or -iname "*.hpp" -or -iname "*.cpp"`


.DEFAULT:
	${TERM} bash -c "${PRELOAD_SCRIPTS}	roslaunch '${LAUNCHDIR}/$@.launch'"
