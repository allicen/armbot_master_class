#!/bin/bash

echo $$ > "$1"
sudo chmod +x scripts/*sh
sed -i -e 's/\r$//' /workspace/scripts/extended/actions/get_data.sh
/workspace/scripts/extended/actions/get_data.sh "$2" "$3"