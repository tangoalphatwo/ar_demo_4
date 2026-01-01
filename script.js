let video = document.getElementById("camera");
let cvCanvas = document.getElementById("cvCanvas");
let ctx = cvCanvas.getContext("2d");

navigator.mediaDevices.getUserMedia({ video: { facingMode: "environment" } })
.then(stream => {
    video.srcObject = stream;

    video.onloadedmetadata = () => {
        cvCanvas.width = video.videoWidth;
        cvCanvas.height = video.videoHeight;

        startAR();
    };
});

function startAR() {
    const src = new cv.Mat(video.videoHeight, video.videoWidth, cv.CV_8UC4);
    const gray = new cv.Mat();
    const edges = new cv.Mat();

    function loop() {
        ctx.drawImage(video, 0, 0, cvCanvas.width, cvCanvas.height);
        let frame = ctx.getImageData(0, 0, cvCanvas.width, cvCanvas.height);

        src.data.set(frame.data);

        cv.cvtColor(src, gray, cv.COLOR_RGBA2GRAY);
        cv.GaussianBlur(gray, gray, new cv.Size(5, 5), 0);
        cv.Canny(gray, edges, 50, 150);

        // TODO: contour detection + quad detection
        // For now, show edges
        ctx.putImageData(new ImageData(edges.data, edges.cols, edges.rows), 0, 0);

        requestAnimationFrame(loop);
    }

    loop();
}
