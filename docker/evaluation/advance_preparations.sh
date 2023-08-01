mkdir -p output
cp -r ../aichallenge/mapfile ./
tar zcvf aichallenge_submit.tar.gz -C ../aichallenge/aichallenge_ws/src aichallenge_submit
