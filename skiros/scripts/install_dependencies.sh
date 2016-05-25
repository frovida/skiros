
raptor='raptor2-2.0.15'
rasqal='rasqal-0.9.33'
redland='redland-1.0.17'

sudo apt-get install libdb-dev

wget "http://download.librdf.org/source/${raptor}.tar.gz"
wget "http://download.librdf.org/source/${rasqal}.tar.gz"
wget "http://download.librdf.org/source/${redland}.tar.gz"

tar xzf "${raptor}.tar.gz"
tar xzf "${rasqal}.tar.gz"
tar xzf "${redland}.tar.gz"

cd ${raptor} && ./configure && make && sudo make install && cd -
cd ${rasqal} && ./configure && make && sudo make install && cd -
cd ${redland} && ./configure && make && sudo make install && cd -

rm -r ${raptor}*
rm -r ${rasqal}*
rm -r ${redland}*
