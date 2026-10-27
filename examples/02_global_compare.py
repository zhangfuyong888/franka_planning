"""调用真实 CLI 的固定场景全局与局部对照。"""
import subprocess,sys
subprocess.run([sys.executable,'-m','panda_planning.benchmark','--suite','tutorial','--trace','--headless','--output','outputs/tutorial-benchmark'],check=True)
