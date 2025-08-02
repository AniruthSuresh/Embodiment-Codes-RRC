## Mask Refinement Pipeline (SAM-Based)

This repository contains a script that performs mask refinement using the Segment Anything Model (SAM).  
The process improves a coarse initial mask by leveraging SAM's point-based predictions.

>  **Tested with:** SAM v1, SAM v2  
>  **Note:** SAM v2 integration added via `main_sam2_run.py` following official installation.

---

###  Setup Instructions

1. **Install SAM v1 (Optional)**  
   Follow the official SAM v1 installation:  
   https://github.com/facebookresearch/segment-anything  
   Use the provided `env.yml` file and ensure you're using Python **3.8.20**.

2. **Install SAM v2**  
   Clone and install SAM v2 from:  
   https://github.com/facebookresearch/sam2  
   Follow the environment setup as described in their README.

---

### Pipeline Overview

- **`main.ipynb`**  
  Demonstrates refinement using SAM v1  
  Key Steps:
  - Initial rough mask is refined using SAM’s point prompts
  - Skeletonization guides point selection toward structural areas

- **`main.py`**  
  Command-line runnable version of the SAM v1 pipeline

- **`main_sam2_run.py`**  
  Extended pipeline using **SAM v2** with the same refinement logic  
  Simply update your image and mask paths in the script to run

---

### Running the Pipeline

- **For SAM v1:**  
  ```bash
  python main.py
  ```
- **For SAM v2:**  
  ```bash
    python main_sam2_run.py
  ```

There is no major difference in the final overlay of the refined mask between SAM v1 and SAM v2 ,  both yield similar visual results in practice.
