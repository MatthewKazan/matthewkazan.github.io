# Matthew Kazan

[Engineering portfolio](https://matthewkazan.github.io/) with a browser SLAM demo, computer vision experiments, and robotics and hardware work.

## Run locally

Run `npm run dev` or `python3 -m http.server 53681 --bind 127.0.0.1` from the root and visit [localhost:53681](http://localhost:53681). GitHub Pages serves the same static files from the root of `main`.

## Mapping demo

- **Replay** renders 30 synthetic RGB-D samples of a room with accumulated translation drift and reliable heading estimates. GTSAM 4.3.0 runs in a Web Worker and optimizes a Pose3 graph with a loop-closure factor. The replay shows the drifted map, animates the samples into their optimized poses, and fuses overlapping samples in 4 cm voxels. Select **Before closure** or **After GTSAM** to compare both reconstructions and the start-to-end gap. The map supports WebGL 2 and Canvas 2D rendering.
- **Camera** starts downloading the 141 MB [MoGe-2 Small model](https://huggingface.co/Ruicheng/moge-2-vits-normal-onnx) when the homepage loads and reuses the browser cache on return visits. Recording needs camera permission on HTTPS or localhost. Auto samples at up to 4 fps, and the rate selector sets a lower or fixed ceiling. Processing can slow the achieved rate. ONNX Runtime Web estimates depth in a worker, ICP aligns overlapping views, and GTSAM optimizes the graph when the camera returns to a matching viewpoint. The browser keeps the camera frames in memory. Model and runtime downloads use Hugging Face and jsDelivr.
- The [original iPhone LiDAR project](https://github.com/MatthewKazan/iPhone-lidar-slam-playground) uses sensor depth and ROS 2.

## Credits

- [Three.js](https://github.com/mrdoob/three.js) 0.180.0 and [bare-wasm GTSAM](https://github.com/bare-wasm/gtsam) 4.3.0 are vendored with their licenses. The OrbitControls import is relative for direct browser loading.
- [Image to Mesh Web](https://github.com/tomosud/Image_to_Mesh_web) provides the MIT-licensed MoGe depth post-processing. [Microsoft MoGe](https://github.com/microsoft/MoGe) provides the model. [ONNX Runtime Web](https://onnxruntime.ai/docs/get-started/with-javascript/web.html) executes inference.
- Learned Visual Pose video and plots by Matthew Kazan. [TartanAir](https://theairlab.org/tartanair-dataset/#term-of-use) frames by Wang et al., [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). The matching figure was resized and converted to WebP.
- Twelve lab photos from Heidi Lim, *iScience* (2021), [figures 2 and 3](https://pmc.ncbi.nlm.nih.gov/articles/PMC8237472/#dfig2), [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). The photos were converted to WebP.
- [Array](https://locusrobotics.com/locusone/fleet/locus-array) photo by Locus Robotics.
