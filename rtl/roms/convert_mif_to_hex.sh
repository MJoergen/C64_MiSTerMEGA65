MIF2HEX=/opt/altera/21.1/quartus/bin/mif2hex
for i in *.mif ../iec_drive/*.mif; do \
   echo "Processing" $i; \
   $MIF2HEX $i tmp.tmp; \
   cat tmp.tmp | cut -c 10-11 | head -n -1 > $i.hex; \
   rm tmp.tmp; \
done;

