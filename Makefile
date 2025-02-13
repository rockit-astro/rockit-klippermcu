RPMBUILD = rpmbuild --define "_topdir %(pwd)/build" \
        --define "_builddir %{_topdir}" \
        --define "_rpmdir %{_topdir}" \
        --define "_srcrpmdir %{_topdir}" \
        --define "_sourcedir %(pwd)"

SRC = chelper/msgblock.c chelper/pollreactor.c chelper/pyhelper.c chelper/serialqueue.c

lib: $(SRC)
	gcc -Wall -g -O2 -shared -fPIC -flto -fwhole-program -fno-use-linker-plugin -o libklippermcu.so $(SRC)

package:
	@mkdir -p build
	@date --utc +%Y%m%d%H%M%S > VERSION
	@${RPMBUILD} --define "_version %(cat VERSION)" -ba python3-rockit-klippermcu.spec
	@mv build/*/*.rpm .
	@rm -rf build VERSION

install: lib
	@date --utc +%Y%m%d%H%M%S > VERSION
	@python3 -m build --outdir .
	@sudo pip3 install rockit.klippermcu-$$(cat VERSION)-py3-none-any.whl
	@sudo mv libklippermcu.so /usr/lib64/
	@rm VERSION
