import sys

import json
sys.path.append('./modules')
f=open("arena_mapper.json","r")
data=json.loads(f.read())
