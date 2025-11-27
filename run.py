import os
import subprocess

def main():
    base_env = os.environ.copy()

    env_domain2 = base_env.copy()
    env_domain2["ROS_DOMAIN_ID"] = "2"

    env_domain10 = base_env.copy()
    env_domain10["ROS_DOMAIN_ID"] = "10"


    p_domain2 = subprocess.Popen(
        ["python3", "bridge_d2.py"],  
        env=env_domain2
    )

    p_domain10 = subprocess.Popen(
        ["python3", "bridge_d10.py"],  
        env=env_domain10
    )

    try:
        p_domain2.wait()
        p_domain10.wait()
    except KeyboardInterrupt:
        p_domain2.terminate()
        p_domain10.terminate()

if __name__ == "__main__":
    main()