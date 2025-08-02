## Mask Refinement Pipeline (SAM-Based)

This repository contains a script that performs mask refinement using the Segment Anything Model (SAM).  
The process improves a coarse initial mask by leveraging SAM's point-based predictions 

> **Tested with:** SAM v1  
>  **TODO:** Extend and validate the refinement pipeline with **SAM v2**.

---

###  Setup Instructions

1. **Install SAM and Dependencies**  
   Follow the official SAM installation steps:  
   https://github.com/facebookresearch/segment-anything?tab=readme-ov-file  
   Create a new Conda environment using the provided `env.yml` file.  
   Make sure you're using Python **3.8.20** for compatibility.

2. **Understand the Pipeline**  
   Open and explore `main.ipynb`. This notebook outlines the complete flow:
   - How refinement is applied on top of a rough mask
   - How skeletonization helps focus refinement on key structural areas

3. **Running the Script**  
   - Set the paths for your input RGB image and the corresponding initial mask  
   - Download the appropriate SAM checkpoint as per the instructions  
   - Run the script using `main.py`  

   All additional implementation details and configuration options are documented within the code itself.

---
