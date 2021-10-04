use opencv::{
	highgui,
	prelude::*,
	Result,
	videoio,
};

fn main() -> Result<()> {
	let window = "video capture";
	highgui::named_window(window, highgui::WINDOW_AUTOSIZE)?;
	#[cfg(ocvrs_opencv_branch_32)]
	let mut cam = videoio::VideoCapture::new_default(0)?; // 0 is the default camera
	#[cfg(not(ocvrs_opencv_branch_32))]
	let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY)?; // 0 is the default camera
	let opened = videoio::VideoCapture::is_opened(&cam)?;
	if !opened {
		panic!("Unable to open default camera!");
	}
	loop {
		let mut frame = Mat::default();
        highgui::set_mouse_callback(window, Some(Box::new(|x,y,z,_| {
            if x == 1 {
                println!("{},{}",y,z);
            }
        })))?;
		cam.read(&mut frame)?;
		if frame.size()?.width > 0 {
			highgui::imshow(window, &mut frame)?;
		}
		let key = highgui::wait_key(10)?;
		if key > 0 && key != 255 {
			break;
		}
	}
	Ok(())
}
