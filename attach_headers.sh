tmp=${1}_with_headers
cp headers.csv $tmp
cat $1 >> $tmp
tr '\r' '\n' < $tmp > ${tmp}_fixed
mv ${tmp}_fixed $tmp
