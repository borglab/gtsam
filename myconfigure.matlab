../configure --prefix=$HOME \
	--with-toolbox=$HOME/toolbox \
	--enable-build-toolbox \
	--enable-install-matlab-tests \
	--enable-install-matlab-examples \
	--enable-install-wrap \
	--with-wrap=$HOME/bin \
	CFLAGS="-fno-inline -g -Wall" \
	CXXFLAGS="-fno-inline -g -Wall" \ 
	LDFLAGS="-fno-inline -g -Wall" \
	--disable-static  \
	--disable-fast-install
