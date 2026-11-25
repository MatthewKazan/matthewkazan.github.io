# Matthew Kazan

[Engineering portfolio](https://matthewkazan.github.io/) with a browser SLAM demo, computer vision experiments, and robotics and hardware work.

## Run locally

Run `npm run dev` or `python3 -m http.server 53681 --bind 127.0.0.1` from the root and visit [localhost:53681](http://localhost:53681). GitHub Pages serves the same static files from the root of `main`.

## Mapping demo

- **Replay** feeds 216 simulated depth scans from a full turn around the room to a browser port of the original ICP and Scan Context pipeline. The worker registers each scan against the local map with point-to-point ICP at a 2 cm voxel size. It assembles submaps, ranks older matches with the original 60 × 20 Scan Context descriptor at a 0.90 cosine threshold, and verifies candidates with two passes of point-to-plane ICP. Verified matches become robust Pose3 factors in GTSAM 4.3.0 WebAssembly. The replay uses an 18,000-voxel submap threshold for the simulated scans. The original recorded phone scans use 75,000. The simulator uses camera poses to produce depth and the sensor preview. The registration and loop detector receive the resulting point clouds.
- **Before closure** shows the trajectory from ICP odometry factors. **After GTSAM** shows the optimized poses. Both maps fuse overlapping samples into 4 cm voxels for rendering. The loop overlay shows the descriptor similarity and ICP fitness for the accepted match. The map supports WebGL 2 and Canvas 2D rendering.
- **Camera** starts downloading the 141 MB [MoGe-2 Small model](https://huggingface.co/Ruicheng/moge-2-vits-normal-onnx) when the homepage loads and reuses the browser cache on return visits. Recording needs camera permission on HTTPS or localhost. Like the iPhone app, Auto samples at 4 fps. The rate selector also supports slower capture. The browser stores the sampled RGB frames as JPEGs in a FIFO while ONNX Runtime Web estimates depth in a worker. Stop recording releases the camera and lets the queue finish processing while the page is open. Camera uses the native multiscale point-to-plane ICP settings. It starts from the last registered pose and uses 10 cm voxels with a 40 cm correspondence radius for the coarse pass, then 2 cm voxels with a 5 cm radius for the fine pass. It rejects low-overlap alignments and uses a 10,000-voxel submap threshold for inferred depth. Scan Context, loop verification, and GTSAM run on the resulting submaps. On phones, Start recording also asks for motion access. With it, each frame's rotation comes from the gyro and ICP fits only the translation, because inferred depth is not consistent enough for ICP to recover turns. Loop checks then run on every submap, and verification keeps the gyro rotation, searches a coarse translation grid for the drifted offset, and requires 50% overlap. Without motion data, Camera falls back to ICP alone. Model and runtime downloads use Hugging Face and jsDelivr.
- The [original iPhone LiDAR project](https://github.com/MatthewKazan/iPhone-lidar-slam-playground) uses sensor depth and ROS 2.

## Credits

- [Three.js](https://github.com/mrdoob/three.js) 0.180.0 and [bare-wasm GTSAM](https://github.com/bare-wasm/gtsam) 4.3.0 are vendored with their licenses. The OrbitControls import is relative for direct browser loading.
- The replay frontend ports the registration settings and Scan Context implementation from [iPhone LiDAR SLAM](https://github.com/MatthewKazan/iPhone-lidar-slam-playground). The ICP convergence and fitness calculations follow [Open3D 0.17](https://github.com/isl-org/Open3D/blob/v0.17.0/cpp/open3d/pipelines/registration/Registration.cpp).
- [Image to Mesh Web](https://github.com/tomosud/Image_to_Mesh_web) provides the MIT-licensed MoGe depth post-processing. [Microsoft MoGe](https://github.com/microsoft/MoGe) provides the model. [ONNX Runtime Web](https://onnxruntime.ai/docs/get-started/with-javascript/web.html) executes inference.
- Learned Visual Pose video and plots by Matthew Kazan. [TartanAir](https://theairlab.org/tartanair-dataset/#term-of-use) frames by Wang et al., [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). The matching figure was resized and converted to WebP.
- Twelve lab photos from Heidi Lim, *iScience* (2021), [figures 2 and 3](https://pmc.ncbi.nlm.nih.gov/articles/PMC8237472/#dfig2), [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). The photos were converted to WebP.
- [Array](https://locusrobotics.com/locusone/fleet/locus-array) photo by Locus Robotics.
