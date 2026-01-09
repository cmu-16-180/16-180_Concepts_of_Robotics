# 16-180 Concepts of Robotics

Welcome to Concepts of Robotics.  We will be working with the Mujoco simulator---a popular simulator used in Robotics and AI research. This repository includes the initial software setup you'll need for this class. We will be using a Git-based workflow. You will clone a main "Course Shell" folder, and then clone individual assignments into that folder as the semester progresses.

**Supported OS:** Windows 10/11, macOS (Intel/Apple Silicon), Ubuntu Linux.


## Step 1: Install Visual Studio Code (VS Code)

1. Download and install from code.visualstudio.com.
2. Open VS Code.
3. **macOS Users:** If asked "Allow Electron to find devices?", click **Allow.**
4. Install the Python extension (by Microsoft) via the Extensions tab on the left. (It may already be installed)

## Step 2: Install Git

You need Git to download course materials and submit assignments.

* **Windows:** Download "Git for Windows" from git-scm.com.
    * During install, accept all default options.
* **macOS:** Open a terminal and type `git --version`. If not installed, it will prompt you to install the Apple Command Line Tools. Accept.
* **Linux:** `sudo apt update && sudo apt install git`

## Step 3: Install Python

We recommend **Python 3.11**.

* **Windows:** python.org/downloads. **CRITICAL:** Check **"Add Python to PATH"** before clicking Install.
* **macOS:** Download the universal installer from [python.org], or use, if you have Homebrew, use `brew install python@3.11.`

## Step 4: Clone the This Repo

This will create your main folder for the semester. Using Git from VSCode.

1. Open VS Code.
2. Click "Clone Git Repositry..."
3. Where it asks "Provide repositry URL or pick a repository source." Copy and paste this URL:
```https://github.com/cmu-16-180/16-180_Concepts_of_Robotics.git```
5. It will pop open a directory selector. Navigate to where you want the class folder (e.g., Documents) and click "Select as Repository Destination".
6. Wait for the download to finish. A popup will appear at the bottom right asking "Would you like to open the cloned repository?"
    * **Click "Open".**

Your file explorer on the left should now look like this:
```
16-180_Concepts_of_Robotics/
├── .gitignore
├── requirements.txt
├── scripts/
│   ├── check_setup.py
│   └── setup_menagerie.py
└── ...
```

## Step 5: Python Virtual Environment

We will create one virtual environment for the entire semester. A virtual environment includes all the software (e.g. the Mujoco Simulator) you will need for the class.

1. In the VS Code Terminal (ensure you are inside 16180_Concepts), run:
    * **Windows:** `python -m venv venv`
    * **Mac/Linux:** `python3 -m venv venv`
2. Activate it:
    * If VS Code asks "We noticed a new environment...," click **Yes.**
    * Check your terminal prompt. It should start with `(venv)`.
    * *If not active:*
        * **Windows:** `.\venv\Scripts\activate`
        * **Mac/Linux:** `source venv/bin/activate`
3. Install Dependencies:
```
pip install --upgrade pip
pip install -r requirements.txt
```

## Step 6: Verify Installation

Run the verification script included in the shell.
* **Windows/Linux:** `python scripts/check_setup.py`
* **macOS:** `mjpython scripts/check_setup.py`

*Success Criteria:* A simulation window appears with a red box falling on a floor.

## Step 7: Install the Robot Zoo (Menagerie)

We will use Git to download the library of robot models.

Run the following command in your terminal (ensure you are still in the 16180_Concepts folder):

```
git clone https://github.com/google-deepmind/mujoco_menagerie.git menagerie
```

This will clone the MuJoCo Menagerie into a menagerie folder within your course directory.






