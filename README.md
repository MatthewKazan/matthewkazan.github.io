# Matthew Kazan

[Engineering portfolio](https://matthewkazan.github.io/) with a browser SLAM demo, computer vision experiments, and robotics and hardware work.

## Run locally

Run `npm run dev` or `python3 -m http.server 53681 --bind 127.0.0.1` from the root and visit [localhost:53681](http://localhost:53681). GitHub Pages serves the same static files from the root of `main`.

## Edit the portfolio

The copy is plain HTML. Change the text inside the heading and paragraph tags in the relevant file, then refresh the local preview.

| Copy | File |
| --- | --- |
| Homepage summaries and order | [`index.html`](index.html), under the **Projects**, **Experience**, and **Background** comments below the demo |
| Project detail pages | `projects/<project>/index.html`, for example [Learned Visual Pose](projects/learned-visual-pose/index.html) |
| Work experience pages | `work/<company>/index.html`, for example [Locus Robotics](work/locus/index.html) |
| Background pages | [About](about/index.html), [Education](education/index.html), [Skills](skills/index.html), [Interests](interests/index.html) |
| Portfolio page layout | [`portfolio.css`](portfolio.css) |

Homepage summaries and detail pages are edited separately. Each detail page keeps its copy inside one `article` with a heading for each topic. The browser tab title and search description are at the top of the file.

## Mapping demo

- **Replay** feeds 216 simulated depth scans from a full turn around the room to a browser port of the original ICP and Scan Context pipeline. The worker registers each scan against the local map with point-to-point ICP at a 2 cm voxel size. It assembles submaps, ranks older matches with the original 60 × 20 Scan Context descriptor at a 0.90 cosine threshold, and verifies candidates with two passes of point-to-plane ICP. Verified matches become robust Pose3 factors in GTSAM 4.3.0 WebAssembly. The replay uses an 18,000-voxel submap threshold for the simulated scans. The original recorded phone scans use 75,000. The simulator uses camera poses to produce depth and the sensor preview. The registration and loop detector receive the resulting point clouds.
- **Before closure** shows the trajectory from ICP odometry factors. **After GTSAM** shows the optimized poses. Both maps fuse overlapping samples into 4 cm voxels for rendering. The loop overlay shows the descriptor similarity and ICP fitness for the accepted match. The map supports WebGL 2 and Canvas 2D rendering.
- **Camera** starts downloading the 141 MB [MoGe-2 Small model](https://huggingface.co/Ruicheng/moge-2-vits-normal-onnx) when the homepage loads and reuses the browser cache on return visits. Recording needs camera permission on HTTPS or localhost. Like the iPhone app, Auto samples at 4 fps. The rate selector also supports slower capture. A service worker (`coi-sw.js`) adds cross-origin isolation headers, which GitHub Pages cannot send, so CPU inference runs on up to 4 threads instead of 1, about 4 times faster. The first visit reloads once to install it. The browser stores the sampled RGB frames as JPEGs in a FIFO while ONNX Runtime Web estimates depth in a worker. Stop recording releases the camera and lets the queue finish processing while the page is open. Upload video maps a recorded clip the same way, sampling it at the selected rate. Camera mode is a rough approximation of the project: depth estimated from one image has a scale that drifts by tens of percent between frames and a focal length that is often 15–30% off, so ICP between consecutive inferred scans trades turns for translation. Instead, `src/visual-odometry.js` tracks Shi-Tomasi corners with pyramidal Lucas–Kanade, solves each frame's pose and depth scale against the latest keyframe (Sim(3) RANSAC, then a reprojection fit), and on phones keeps the rotation from the gyro. The first turning frames calibrate the focal length, and the worker rebuilds the map with it. The scaled clouds, without points on depth edges or beyond 5 m (the LiDAR's range), feed the same submap, Scan Context, and GTSAM pipeline with a 10,000-voxel submap threshold. Each submap keeps BRIEF features of its first image. Older submaps that faced nearly the same way become loop candidates, and matching their images measures the loop, since ICP between two inferred submaps settles tens of centimetres off. On rendered HM3D sequences this keeps spin trajectories within about 3–9 cm after GTSAM. After a loop closes, Before closure and After GTSAM compare the odometry and optimized maps, as in the replay. Model and runtime downloads use Hugging Face and jsDelivr.
- The [original iPhone LiDAR project](https://github.com/MatthewKazan/iPhone-lidar-slam-playground) uses sensor depth and ROS 2.

## Credits

- [Three.js](https://github.com/mrdoob/three.js) 0.180.0 and [bare-wasm GTSAM](https://github.com/bare-wasm/gtsam) 4.3.0 are vendored with their licenses. The OrbitControls import is relative for direct browser loading.
- The replay frontend ports the registration settings and Scan Context implementation from [iPhone LiDAR SLAM](https://github.com/MatthewKazan/iPhone-lidar-slam-playground). The ICP convergence and fitness calculations follow [Open3D 0.17](https://github.com/isl-org/Open3D/blob/v0.17.0/cpp/open3d/pipelines/registration/Registration.cpp).
- [Image to Mesh Web](https://github.com/tomosud/Image_to_Mesh_web) provides the MIT-licensed MoGe depth post-processing. [Microsoft MoGe](https://github.com/microsoft/MoGe) provides the model. [ONNX Runtime Web](https://onnxruntime.ai/docs/get-started/with-javascript/web.html) executes inference.
- Learned Visual Pose video and plots by Matthew Kazan. [TartanAir](https://theairlab.org/tartanair-dataset/#term-of-use) frames by Wang et al., [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). The matching figure was resized and converted to WebP.
- Twelve lab photos from Heidi Lim, *iScience* (2021), [figures 2 and 3](https://pmc.ncbi.nlm.nih.gov/articles/PMC8237472/#dfig2), [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). The photos were converted to WebP.
- [Array](https://locusrobotics.com/locusone/fleet/locus-array) photo by Locus Robotics.
