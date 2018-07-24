declare -A nodes
declare -A edges
for f in $(cat cscope.files); do
  lines=$(cat $f | sed -e ":join /^class [^{;]*$/{N
    s/\\n/ /g
    b join
    }" |
    grep "^class.*{")
  [ "$lines" = "" ] && continue
  while read -r aline; do
    line=$(echo ${aline} |
      egrep -iv "interrupt|DEPRECATED|UnlimitedHandleResource" |
      sed "s/^class //" |
      sed "s/virtual //g" |
      sed "s/public //g" |
      sed "s/final //g" |
      sed "s/::/__/g" |
      sed "s/{.*//" |
      sed "s/  *//g")

    if [ "$line" != "" ]; then
      if [[ $line != *":"* ]]; then
        dst=$line
        src=""
      else
        dst=$(echo $line | sed "s/\([^:]*\):\(.*\)/\1/" | sed "s/  *//g")
        src=$(echo $line | sed "s/\([^:]*\):\(.*\)/\2/" | sed "s/  *//g")
      fi
      nodes[${dst}]="$src"
    fi
  done <<< "$lines"
done
echo "Title Classes"
# add src nodes to nodes array
declare -A srcnodes
for dst in "${!nodes[@]}"; do
  unset s
  declare -a s
  IFS=',' read -ra s <<< ${nodes[${dst}]}
  for src in "${s[@]}"; do
    if ! [ ${nodes[${src}]+isset} ]; then
      srcnodes[${src}]=1
    fi
  done
done
for dst in "${!nodes[@]}"; do
  t=""
  case $dst in
    *PID*) t="<<s>>" ;;
  esac
  case $dst in
    *Robot*) t="<<p>>" ;;
    *Drive) t="<<p>>" ;;
  esac
  as=$(echo ${dst} | sed "s/__/::/g")
  echo "state $dst as \"${as}\"$t"
done
for src in "${!srcnodes[@]}"; do
  as=$(echo ${src} | sed "s/__/::/g")
  echo "state $src as \"${as}\"$t"
done
for dst in "${!nodes[@]}"; do
  unset s
  declare -a s
  IFS=',' read -ra s <<< ${nodes[${dst}]}
  for src in "${s[@]}"; do
    echo "$src --> $dst"
  done
done
