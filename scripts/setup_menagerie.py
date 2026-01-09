import os
import subprocess
import shutil

def main():
    target_dir = "menagerie"
    repo_url = "https://github.com/google-deepmind/mujoco_menagerie.git"
    
    # Robots we want to download (add more here if needed later)
    # The Unitree Go1 is the dog we use for demos.
    robots_to_include = ["unitree_go1", "shadow_hand"]

    print(f"--- Setting up MuJoCo Menagerie in './{target_dir}' ---")
    print(f"Targeting specific robots: {robots_to_include}")

    # Check for Git
    if shutil.which("git") is None:
        print("[ERROR] Git is not found. Please install Git.")
        return

    # Clean previous install
    if os.path.exists(target_dir):
        print(f"Directory '{target_dir}' already exists.")
        choice = input("Do you want to delete it and re-clone? (y/N): ")
        if choice.lower() == 'y':
            shutil.rmtree(target_dir)
        else:
            print("Skipping setup.")
            return

    try:
        # 1. Clone with "blobless" filter (downloads almost no data initially)
        # --sparse initializes the sparse-checkout file immediately
        print("Initializing sparse clone...")
        subprocess.check_call([
            "git", "clone", 
            "--filter=blob:none", 
            "--sparse", 
            repo_url, target_dir
        ])

        # 2. Configure sparse checkout to only include specific folders
        cwd = os.getcwd()
        os.chdir(target_dir) # Step into the folder
        
        print("Configuring sparse checkout...")
        cmd = ["git", "sparse-checkout", "set"] + robots_to_include
        subprocess.check_call(cmd)
        
        # 3. Go back to root
        os.chdir(cwd)
        
        print("\n[SUCCESS] Menagerie installed (Sparse Mode).")
        print(f"Only {robots_to_include} were downloaded.")

    except subprocess.CalledProcessError as e:
        print(f"\n[ERROR] Git operation failed: {e}")
        print("Ensure you have a recent version of Git installed.")

if __name__ == "__main__":
    main()