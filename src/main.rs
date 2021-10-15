use opencv::{calib3d, core, highgui, prelude::*, videoio, Result};

//fn main() -> Result<()> {
//	let window = "video capture";
//	highgui::named_window(window, highgui::WINDOW_AUTOSIZE)?;
//	#[cfg(ocvrs_opencv_branch_32)]
//	let mut cam = videoio::VideoCapture::new_default(0)?; // 0 is the default camera
//	#[cfg(not(ocvrs_opencv_branch_32))]
//	let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY)?; // 0 is the default camera
//	let opened = videoio::VideoCapture::is_opened(&cam)?;
//	if !opened {
//		panic!("Unable to open default camera!");
//	}
//	loop {
//		let mut frame = Mat::default();
//        highgui::set_mouse_callback(window, Some(Box::new(|x,y,z,_| {
//            if x == 1 {
//                println!("{},{}",y,z);
//            }
//        })))?;
//		cam.read(&mut frame)?;
//		if frame.size()?.width > 0 {
//			highgui::imshow(window, &mut frame)?;
//		}
//		let key = highgui::wait_key(10)?;
//		if key > 0 && key != 255 {
//			break;
//		}
//	}
//	Ok(())
//}
fn main() -> Result<()> {
    let camera_matrix =
        Mat::from_slice_2d(&[[824.0, 0.0, 251.0], [0.0, 825.9, 286.5], [0.0, 0.0, 1.0]])?;
    let dist_coeffs = Mat::from_slice_2d(&[[0.23233, -0.99375, 0.00160, 0.00145, 0.00000]])?;
    let camera_matrix2 =
        Mat::from_slice_2d(&[[853.0, 0.0, 217.0], [0.0, 852.9, 269.5], [0.0, 0.0, 1.0]])?;
    let dist_coeffs2 = Mat::from_slice_2d(&[[0.30829, -1.61541, 0.01495, -0.00758, 0.00000]])?;
    // no problem
    let om = Mat::from_slice(&[0.01911, 0.03125, -0.00960])?;
    let mut r = Mat::default();
    let mut temp = Mat::default();
    calib3d::rodrigues(&om, &mut r, &mut temp)?;
    // error
    let t = Mat::from_slice_2d(&[
        [-70.59612],
        [-2.60704],
        [18.87635]
    ])?;
    let size = core::Size2i {
        width: 400,
        height: 200,
    };
    let mut r1 = Mat::default();
    let mut r2 = Mat::default();
    let mut p1 = Mat::default();
    let mut p2 = Mat::default();
    let mut q = Mat::default();
    let mut valid_pix_roi1 = core::Rect::default();
    let mut valid_pix_roi2 = core::Rect::default();
    // Something error
    opencv::calib3d::stereo_rectify(
        &camera_matrix,
        &dist_coeffs,
        &camera_matrix2,
        &dist_coeffs2,
        size,
        &r,
        &t,
        &mut r1,
        &mut r2,
        &mut p1,
        &mut p2,
        &mut q,
        opencv::calib3d::CALIB_ZERO_DISPARITY,
        -1.0,
        size,
        &mut valid_pix_roi1,
        &mut valid_pix_roi2,
    )?;
    let mut left_map1 = Mat::default();
    let mut left_map2 = Mat::default();
    opencv::calib3d::init_undistort_rectify_map(
        &camera_matrix,
        &dist_coeffs,
        &r1,
        &p1,
        size,
        core::CV_16SC2,
        &mut left_map1,
        &mut left_map2,
    )?;
    let mut right_map1 = Mat::default();
    let mut right_map2 = Mat::default();
    opencv::calib3d::init_undistort_rectify_map(
        &camera_matrix2,
        &dist_coeffs2,
        &r2,
        &p2,
        size,
        core::CV_16SC2,
        &mut right_map1,
        &mut right_map2,
    )?;
    //println!("{:?}",right_map2.to_vec_2d()? as Vec<Vec<u16>>);
    // deepth
    let mut cam2 = videoio::VideoCapture::new(4, videoio::CAP_V4L)?;
    cam2.set(videoio::CAP_PROP_FRAME_WIDTH, 400.0)?;
    cam2.set(videoio::CAP_PROP_FRAME_HEIGHT, 200.0)?;
    let mut cam3 = videoio::VideoCapture::new(6, videoio::CAP_V4L)?; // 0 is the default camera
    cam3.set(videoio::CAP_PROP_FRAME_WIDTH, 400.0)?;
    cam3.set(videoio::CAP_PROP_FRAME_HEIGHT, 200.0)?;
    let window = "center";
    highgui::named_window(window, highgui::WINDOW_AUTOSIZE)?;
    let mut num: i32 = 0;
    let mut blocksize: i32 = 5;
    highgui::create_trackbar("num", window, Some(&mut num), 10, Some(Box::new(|_| {})))?;
    highgui::create_trackbar(
        "blockSize",
        window,
        Some(&mut blocksize),
        20,
        Some(Box::new(|_| {})),
    )?;
    loop {
        let mut frame1 = Mat::default();
        let mut frame2 = Mat::default();
        cam2.read(&mut frame1)?;
        cam3.read(&mut frame2)?;
        if frame1.size()?.width > 0 && frame2.size()?.width > 0 {
            let mut img1_rectified = Mat::default();
            let mut img2_rectified = Mat::default();
            //println!("test");
            //Here, remap has some trouble, after remap , It is really black . I don't known why.
            //It is hardly to belive.
            opencv::imgproc::remap(
                &frame1,
                &mut img1_rectified,
                &left_map1,
                &left_map2,
                opencv::imgproc::INTER_LINEAR,
                core::BORDER_CONSTANT,
                core::Scalar::default(),
            )?;
            opencv::imgproc::remap(
                &frame2,
                &mut img2_rectified,
                &right_map1,
                &right_map2,
                opencv::imgproc::INTER_LINEAR,
                core::BORDER_CONSTANT,
                core::Scalar::default(),
            )?;

            //println!("test");
            let mut gray_img1 = Mat::default();
            let mut gray_img2 = Mat::default();
            opencv::imgproc::cvt_color(
                &img1_rectified,
                &mut gray_img1,
                opencv::imgproc::COLOR_BGR2GRAY,
                0,
            )?;
            opencv::imgproc::cvt_color(
                &img2_rectified,
                &mut gray_img2,
                opencv::imgproc::COLOR_BGR2GRAY,
                0,
            )?;
            //println!("test");
            let num_local = highgui::get_trackbar_pos("num", window)?;
            let mut blocksize_local = highgui::get_trackbar_pos("blockSize", window)?;
            if blocksize_local % 2 == 0 {
                blocksize_local += 1;
            }
            if blocksize_local < 5 {
                blocksize_local = 5;
            }
            let mut stereo = <dyn calib3d::StereoBM>::create(16 * num_local, blocksize_local)?;
            let mut disparity = Mat::default();
            //println!("test");
            stereo.compute(&gray_img1, &gray_img2, &mut disparity)?;
            let mut disp = Mat::default();
            let mask = Mat::default();
            //println!("test");
            core::normalize(
                &disparity,
                &mut disp,
                0.0,
                255.0,
                core::NORM_MINMAX,
                core::CV_8U,
                &mask,
            )?;
            //let mut output = core::Vector::<Mat>::new();
            let mut output = Mat::default();
            //let mut output2 = Mat::default();
            calib3d::reproject_image_to_3d(&disp, &mut output, &q, true, 3)?;
            //output.convert_to(&mut output2, core::CV_16SC1, 0.0, 0.0)?;
            //let a = output.at_2d(3, 3)? as &(i32,i32,i32);
            //println!("{:?}",a);
            let show: Vec<Vec<core::Vec3s>> = output.to_vec_2d()?;
            highgui::set_mouse_callback(
                window,
                Some(Box::new(move |x, y, z, _| {
                    if x == 1 {
                        println!("{},{}", y, z);
                        println!("{:?}", show[z as usize][y as usize]);
                    }
                })),
            )?;
            highgui::imshow("first", &img1_rectified)?;
            highgui::imshow("second", &img2_rectified)?;
            highgui::imshow(window, &disp)?;
        }
        let key = highgui::wait_key(10)?;
        if key > 0 && key != 255 {
            break;
        }
    }

    Ok(())
}
