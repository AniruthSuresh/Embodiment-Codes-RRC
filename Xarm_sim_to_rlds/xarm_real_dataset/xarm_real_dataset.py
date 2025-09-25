from typing import Iterator, Tuple, Any

import glob,os,json
import numpy as np
import tensorflow as tf
import tensorflow_datasets as tfds
import tensorflow_hub as hub
from PIL import Image


# Command to run : tfds build xarm_real_dataset --data_dir=~/tfds_data --overwrite
class xArmRealDataset(tfds.core.GeneratorBasedBuilder):
    """DatasetBuilder for xArm real dataset."""

    VERSION = tfds.core.Version('1.0.0')
    RELEASE_NOTES = {
      '1.0.0': 'Initial release.',
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._embed = hub.load("https://tfhub.dev/google/universal-sentence-encoder-large/5")
        # tasks_file = os.path.join(
        #     "/home/nitin/Desktop/RRC/data",  # adjust if needed
        #     "current_tasks.json"
        # )
        # with open(tasks_file, "r") as f:
        #     tasks = json.load(f)
        # # Map idx -> instruction
        # self._task_map = {t["idx"]: t["current_task"] for t in tasks}


    def _info(self) -> tfds.core.DatasetInfo:
        """Dataset metadata (homepage, citation,...)."""
        return self.dataset_info_from_configs(
            features=tfds.features.FeaturesDict({
                'steps': tfds.features.Dataset({
                    'observation': tfds.features.FeaturesDict({
                        'image': tfds.features.Image(
                            shape=(None, None, 3),
                            dtype=np.uint8,
                            encoding_format='png',
                            doc='Main camera RGB observation.',
                        ),
                        'state': tfds.features.Tensor(
                            shape=(6,),# <----In case of xarm6 , the joint angles are 6
                            dtype=np.float32,
                            doc='Robot state, consists of [7x robot joint angles, '
                                '2x gripper position, 1x door opening angle].',
                        )
                    }),
                    'action': tfds.features.Tensor(
                        shape=(7,),
                        dtype=np.float32,
                        doc='Robot action, consists of [7x joint velocities, '
                            '2x gripper velocities, 1x terminate episode].',
                    ),
                    'discount': tfds.features.Scalar(
                        dtype=np.float32,
                        doc='Discount if provided, default to 1.'
                    ),
                    'reward': tfds.features.Scalar(
                        dtype=np.float32,
                        doc='Reward if provided, 1 on final step for demos.'
                    ),
                    'is_first': tfds.features.Scalar(
                        dtype=np.bool_,
                        doc='True on first step of the episode.'
                    ),
                    'is_last': tfds.features.Scalar(
                        dtype=np.bool_,
                        doc='True on last step of the episode.'
                    ),
                    'is_terminal': tfds.features.Scalar(
                        dtype=np.bool_,
                        doc='True on last step of the episode if it is a terminal step, True for demos.'
                    ),
                    'language_instruction': tfds.features.Text(
                        doc='Language Instruction.'
                    ),
                    'language_embedding': tfds.features.Tensor(
                        shape=(512,),
                        dtype=np.float32,
                        doc='Kona language embedding. '
                            'See https://tfhub.dev/google/universal-sentence-encoder-large/5'
                    ),
                }),
                'episode_metadata': tfds.features.FeaturesDict({
                    'file_path': tfds.features.Text(
                        doc='Path to the original data file.'
                    ),
                }),
            }))

    def _split_generators(self, dl_manager: tfds.download.DownloadManager):
        """Define data splits."""
        # dataset_path = "/home/nitin/Desktop/RRC/data"
        dataset_path = "/home/nitin/Desktop/synched_traj_npz"
        # dataset_path = "/home/nitin/Desktop/valid_traj"
        # dataset_path = "/home/nitin/Desktop/test_conversion"
        return {
            'train': self._generate_examples(path=dataset_path),
        }

    # def _generate_examples(self, path) -> Iterator[Tuple[str, Any]]:
    #     """Generator of examples for each split."""

    #     def _parse_example(episode_path):
    #         # load raw data --> this should change for your dataset
    #         data = np.load(episode_path, allow_pickle=True)     # this is a list of dicts in our case

    #         # assemble episode --> here we're assuming demos so we set reward to 1 at the end
    #         episode = []
    #         for i, step in enumerate(data):
    #             # compute Kona language embedding
    #             language_embedding = self._embed([step['language_instruction']])[0].numpy()

    #             episode.append({
    #                 'observation': {
    #                     'image': step['image'],
    #                     'wrist_image': step['wrist_image'],
    #                     'state': step['state'],
    #                 },
    #                 'action': step['action'],
    #                 'discount': 1.0,
    #                 'reward': float(i == (len(data) - 1)),
    #                 'is_first': i == 0,
    #                 'is_last': i == (len(data) - 1),
    #                 'is_terminal': i == (len(data) - 1),
    #                 'language_instruction': step['language_instruction'],
    #                 'language_embedding': language_embedding,
    #             })

    #         # create output data sample
    #         sample = {
    #             'steps': episode,
    #             'episode_metadata': {
    #                 'file_path': episode_path
    #             }
    #         }

    #         # if you want to skip an example for whatever reason, simply return None
    #         return episode_path, sample

    #     # create list of all examples
    #     episode_paths = glob.glob(path)

    #     # for smallish datasets, use single-thread parsing
    #     for sample in episode_paths:
    #         yield _parse_example(sample)

    #     # for large datasets use beam to parallelize data parsing (this will have initialization overhead)
    #     # beam = tfds.core.lazy_imports.apache_beam
    #     # return (
    #     #         beam.Create(episode_paths)
    #     #         | beam.Map(_parse_example)
    #     # )


    # def _generate_examples(self, path) -> Iterator[Tuple[str, Any]]:
    #     episode_dirs = sorted(glob.glob(os.path.join(path, "*")))
    #     for ep_dir in episode_dirs:
    #         if not os.path.isdir(ep_dir):
    #             continue
    #         ep_id = os.path.basename(ep_dir)
    #         print(ep_id)

    #         # Load arrays
    #         actions = np.loadtxt(os.path.join(ep_dir, "actions.txt"))[:, :7]
    #         observations = np.loadtxt(os.path.join(ep_dir, "observations.txt"))

    #         # Load images
    #         img_dir = os.path.join(ep_dir, "rgb_images")
    #         img_files = sorted(glob.glob(os.path.join(img_dir, "*")))
    #         images = [np.array(Image.open(f)) for f in img_files]

    #         # Language instruction for this episode
    #         instruction = self._task_map.get(ep_id, "")
    #         embedding = self._embed([instruction])[0].numpy()

    #         T = min(len(actions), len(observations), len(images))
    #         episode = []
    #         for i in range(T):
    #             episode.append({
    #                 'observation': {
    #                     'image': images[i],
    #                     'state': observations[i].astype(np.float32),
    #                 },
    #                 'action': actions[i].astype(np.float32),
    #                 'discount': 1.0,
    #                 'reward': float(i == (T - 1)),
    #                 'is_first': i == 0,
    #                 'is_last': i == (T - 1),
    #                 'is_terminal': i == (T - 1),
    #                 'language_instruction': instruction,
    #                 'language_embedding': embedding,
    #             })

    #         sample = {
    #             'steps': episode,
    #             'episode_metadata': {'file_path': ep_dir},
    #         }
    #         yield ep_id, sample


    def _generate_examples(self, path) -> Iterator[Tuple[str, Any]]:
        """Generator for .npz episodes that already contain episode_metadata and steps."""
        npz_files = sorted(glob.glob(os.path.join(path, "*.npz")))

        for ep_file in npz_files:
            ep_id = os.path.splitext(os.path.basename(ep_file))[0]

            # Load episode dict from .npz
            data = np.load(ep_file, allow_pickle=True)
            episode = data["episode"].item()  # stored as a dict

            steps = episode["steps"]
            metadata = episode["episode_metadata"]

            processed_steps = []
            for i, step in enumerate(steps):
                obs = step["observation"]
                act = step["action"]  # already 7-dim
                # act = np.concatenate([step["action_dict"]["joint_position"], step["action_dict"]["gripper_position"]], axis=-1)
                instr = step.get("language_instruction", "")

                # embed once per episode, not per step
                if i == 0:
                    embedding = self._embed([instr])[0].numpy()

                processed_steps.append({
                    "observation": {
                        "image": obs["exterior_image_1_left"],  # pick your main image
                        "state": np.array(obs["joint_position"], dtype=np.float32),
                    },
                    "action": np.array(act, dtype=np.float32),
                    "discount": float(step.get("discount", 1.0)),
                    "reward": float(step.get("reward", 0.0)),
                    "is_first": bool(step.get("is_first", i == 0)),
                    "is_last": bool(step.get("is_last", i == len(steps) - 1)),
                    "is_terminal": bool(step.get("is_terminal", i == len(steps) - 1)),
                    "language_instruction": instr,
                    "language_embedding": embedding,
                })

            sample = {
                "steps": processed_steps,
                "episode_metadata": {
                    "file_path": metadata.get("file_path", ep_file),
                },
            }

            yield ep_id, sample
