"""PRM cold/warm 成本单列；同场景十个保存查询。"""
from panda_planning.benchmark import multi_query
rows=multi_query('outputs/prm-queries',queries=10)
for row in rows:print(row['query'],row['group'],row['status'],row['stats']['build_time'],row['stats']['query_time'])
