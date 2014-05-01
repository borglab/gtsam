flex -L -l lp_rlp.l
sed -e "s/yy/lp_yy/g" lex.yy.c >lp_rlp.h
del lex.yy.c

bison --no-lines -y lp_rlp.y
sed -e "s/yy/lp_yy/g" y.tab.c >lp_rlp.c
del y.tab.c
