# Matthew Kazan

[Engineering portfolio](https://matthewkazan.github.io/) with a browser SLAM demo, computer vision experiments, and robotics and hardware work.

## Run locally

Run `npm run dev` or `python3 -m http.server 53681 --bind 127.0.0.1` from the root and visit [localhost:53681](http://localhost:53681). GitHub Pages serves the same static files from the root of `main`.

## Mapping demo

- **Replay** renders 30 synthetic RGB-D samples of a room. The camera trajectory has simulated odometry drift. GTSAM 4.3.0 runs in a Web Worker and optimizes a Pose3 graph with a loop-closure factor. Toggle loop closure after the last frame to compare poses.
- **Camera** asks for camera permission on HTTPS or localhost. The first capture downloads the 141 MB [MoGe-2 Small model](https://huggingface.co/Ruicheng/moge-2-vits-normal-onnx). ONNX Runtime Web estimates depth from RGB in a worker. Nearby views are aligned with point-to-point ICP. If the camera revisits a sufficiently overlapping earlier view, GTSAM optimizes the resulting graph. The browser keeps the frames in memory. Model downloads use the public Hugging Face and jsDelivr endpoints.
- The [original iPhone LiDAR project](https://github.com/MatthewKazan/iPhone-lidar-slam-playground) uses sensor depth and ROS 2. Its recorded pose graphs were compared against the browser binding separately. With both native and WASM GTSAM pinned to 4.3.0, 9 of 9 graph solves agreed within numerical precision. The public replay uses synthetic depth and poses.

## Credits

- [Three.js](https://github.com/mrdoob/three.js) 0.180.0 and [bare-wasm GTSAM](https://github.com/bare-wasm/gtsam) 4.3.0 are vendored with their licenses. The OrbitControls import is relative for direct browser loading.
- [Image to Mesh Web](https://github.com/tomosud/Image_to_Mesh_web) provides the MIT-licensed MoGe depth post-processing. [Microsoft MoGe](https://github.com/microsoft/MoGe) provides the model. [ONNX Runtime Web](https://onnxruntime.ai/docs/get-started/with-javascript/web.html) executes inference.
- Learned Visual Pose plots by Matthew Kazan. [TartanAir](https://theairlab.org/tartanair-dataset/#term-of-use) frames by Wang et al., [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). The matching figure was resized and converted to WebP.
- Twelve lab photo from Heidi Lim, *iScience* (2021), [Figure 2](https://pmc.ncbi.nlm.nih.gov/articles/PMC8237472/#dfig2), [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). The photo was converted to WebP.
- Array footage by Locus Robotics via Business Wire. The Array photo is embedded from Locus Robotics’ public LinkedIn post.
