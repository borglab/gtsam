#!/opt/local/bin/gnuplot
reset
#set terminal pdf
set title "Smart range SLAM result"
set size ratio -1
plot "rangeResult.txt" using 2:3 with lines, "rangeResultLM.txt" using 2:3:4 with circles, "rangeResultSR.txt" using 2:3:4 with circles