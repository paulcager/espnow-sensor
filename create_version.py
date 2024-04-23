import os
import subprocess
from datetime import datetime

result = subprocess.run(
    ["git", "rev-parse", "--short", "HEAD"],
    stdout=subprocess.PIPE,
    text=True
)
commit_hash = result.stdout.strip()

result = subprocess.run(
    ["git", "describe", "--tags", "--dirty", "--always"],
    stdout=subprocess.PIPE,
    text=True
)
tag = result.stdout.strip()

dirty_check = subprocess.run(
    ["git", "status", "--porcelain"],
    stdout=subprocess.PIPE,
    text=True
)
is_dirty = "-dirty" if dirty_check.stdout.strip() else ""

branch_result = subprocess.run(
    ["git", "rev-parse", "--abbrev-ref", "HEAD"],
    stdout=subprocess.PIPE,
    text=True
)
branch_name = branch_result.stdout.strip()

current_timestamp = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
current_directory = os.path.basename(os.getcwd())

define_output = (
    f"#define APP \"{current_directory}\"\n"
    f"#define VERSION \"{tag}\"\n"
    f"#define VERSION_COMMIT_HASH \"{commit_hash}{is_dirty}\"\n"
    f"#define VERSION_TIMESTAMP \"{current_timestamp}\"\n"
    f"#define VERSION_BRANCH \"{branch_name}\"\n"
    f"#define APP_BANNER APP \" v\" VERSION \" (\" VERSION_COMMIT_HASH \") \" VERSION_TIMESTAMP\n"
)

with open("include/version.h", "w") as file:
    file.write(define_output)
