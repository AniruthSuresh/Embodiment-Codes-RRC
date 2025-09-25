# xArm Real Dataset - RLDS Converter

This code converts xArm simulation data into RLDS (Reinforcement Learning Datasets) format for robotics research and training.

## Code Summary

The `xArmRealDataset` class processes robot simulation episodes containing:

- RGB camera images
- Robot joint states and actions
- Language instructions mapped from task definitions
- Generates universal sentence embeddings for language instructions

## Setup & Installation

1. **Create conda environment** (from `rlds_dataset_builder` directory):

   ```bash
   conda env create -f environment_ubuntu.yml
   conda activate rlds_env
   ```
2. **Install required packages**:

   ```bash
   pip install tensorflow tensorflow_datasets tensorflow_hub apache_beam matplotlib plotly
   ```

## Configuration

### Data Path Configuration

Change the dataset path in `xarm_real_dataset.py`:

```python
# Line 88-91: Update these paths
dataset_path = "/home/nitin/Desktop/RRC/data"  # Main data directory
tasks_file = os.path.join(
    "/home/nitin/Desktop/RRC/data",  # Path to current_tasks.json
    "current_tasks.json"
)
```

### Expected Data Structure

```
data/
├── current_tasks.json          # Task definitions
├── 1/                         # Episode directories
│   ├── actions.txt            # Action sequences
│   ├── observations.txt       # State observations  
│   └── rgb_images/           # Camera images
├── 2/
└── ...
```

## Dataset Metadata Format

### Features Structure

```python
{
    'steps': {
        'observation': {
            'image': Image(shape=(H, W, 3), dtype=uint8)        # RGB camera feed
            'state': Tensor(shape=(7,), dtype=float32)          # Robot joint angles
        },
        'action': Tensor(shape=(7,), dtype=float32),            # Joint velocities
        'discount': Scalar(dtype=float32),                      # Always 1.0
        'reward': Scalar(dtype=float32),                        # 1.0 on last step
        'is_first': Scalar(dtype=bool),                         # True for first step
        'is_last': Scalar(dtype=bool),                          # True for last step  
        'is_terminal': Scalar(dtype=bool),                      # True for last step
        'language_instruction': Text(),                         # Task description
        'language_embedding': Tensor(shape=(512,), dtype=float32) # USE embeddings
    },
    'episode_metadata': {
        'file_path': Text()                                     # Original episode path
    }
}
```

### Data Dimensions

- **Images**: Variable size RGB images (H×W×3, uint8)
- **State**: 7-dimensional robot joint angles (float32)
- **Action**: 7-dimensional joint velocities (float32)
- **Language Embedding**: 512-dimensional Universal Sentence Encoder vectors (float32)

## Usage

Build the dataset:

```bash
cd /path/to/xarm_real_dataset/
tfds build xarm_real_dataset --data_dir=~/tfds_data --overwrite
```

## Task Mapping

The code loads task descriptions from `current_tasks.json`:

```json
[
    {"idx": "1", "current_task": "pick up the red block"},
    {"idx": "2", "current_task": "place object in container"}
]
```

Episode directories are mapped to task instructions using their folder names as indices.
