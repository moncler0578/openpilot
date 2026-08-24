import os
import subprocess
from common.basedir import BASEDIR


PREBUILT = os.path.exists(os.path.join(BASEDIR, 'prebuilt'))

class Spinner():
  def __init__(self):
    try:
      self.spinner_proc = subprocess.Popen(["./spinner"],
                                           stdin=subprocess.PIPE,
                                           cwd=os.path.join(BASEDIR, "selfdrive", "ui"),
                                           close_fds=True)
    except OSError:
      self.spinner_proc = None

  def __enter__(self):
    return self

  def update(self, spinner_text: str):
    if self.spinner_proc is not None:
      self.spinner_proc.stdin.write(spinner_text.encode('utf8') + b"\n")
      try:
        self.spinner_proc.stdin.flush()
      except BrokenPipeError:
        pass

  def update_progress(self, cur: float, total: float):
    self.update(str(round(100 * cur / total)))

  def close(self):
    if self.spinner_proc is not None:
      try:
        self.spinner_proc.stdin.close()
      except BrokenPipeError:
        pass
      self.spinner_proc.terminate()
      self.spinner_proc = None

  def __del__(self):
    self.close()

  def __exit__(self, exc_type, exc_value, traceback):
    self.close()


if __name__ == "__main__":
  import time
  if PREBUILT:
    with Spinner() as s:
      # opkr
      s.update("G")
      time.sleep(1.0)
      s.update("GE")
      time.sleep(0.3)
      s.update("GEN")
      time.sleep(0.3)
      s.update("GENE")
      time.sleep(0.3)
      s.update("GENES")
      time.sleep(0.3)
      s.update("GENESI")
      time.sleep(0.3)
      s.update("GENESIS")
      time.sleep(1.0)
      s.update("OPENPILOT")
      time.sleep(1.0)
      s.update("Prebuilt Booting...")
      time.sleep(3.0)
  else:
    with Spinner() as s:
      s.update("Spinner text")
      time.sleep(5.0)
    print("gone")
    time.sleep(5.0)
