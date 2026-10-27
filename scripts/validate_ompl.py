"""在有OMPL真实轮子的环境执行绑定验收；缺包时失败，不把skip当作通过。"""
from pathlib import Path
import importlib.util
import subprocess
import sys

if importlib.util.find_spec('ompl') is None:
    print('OMPL未安装，真实绑定验收未完成。网络可用后在项目虚拟环境执行：\n'
          'python -m pip install ompl==2.0.1\npython scripts/validate_ompl.py', file=sys.stderr)
    raise SystemExit(2)
root = Path(__file__).resolve().parents[1]
result = subprocess.run([sys.executable, '-m', 'pytest', '-q', 'tests/test_planning.py', 'tests/test_ompl_lifecycle.py', '-k', 'real_ompl or prm_query_not_cut or native_callbacks or ompl_budget_expiry'], cwd=root)
raise SystemExit(result.returncode)
