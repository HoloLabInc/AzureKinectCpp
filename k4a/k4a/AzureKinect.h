#pragma once

#include <stdexcept>
#include <memory>
#include <vector>

#include <k4a\k4a.h>

namespace AzureKinect
{
	class Capture;
	class Transformation;

	// Kinectセンサー
	class Sensor
	{
		friend Capture;
		friend Transformation;

		struct SyncJack
		{
			bool in = false;
			bool out = false;

			SyncJack(bool _in, bool _out)
				: in(_in), out(_out)
			{
			}
		};

	public:

		~Sensor()
		{
		}

		void open(int index = 0)
		{
			// Kinectを開く
			{
				k4a_device_t handle = 0;
				auto ret = k4a_device_open(index, &handle);
				if (ret != K4A_RESULT_SUCCEEDED) {
					throw std::runtime_error("k4a_device_open");
				}

				handle_ = std::shared_ptr<_k4a_device_t>(handle, k4a_device_close);
			}

			// シリアルナンバーを取得する。
			{
				// 第二引数のバッファをNULL(0)にすると、シリアルナンバーのバッファサイズが返る
				size_t serial_number_size = 0;
				auto ret = k4a_device_get_serialnum(handle_.get(), 0, &serial_number_size);
				if (ret != K4A_BUFFER_RESULT_TOO_SMALL) {
					throw std::runtime_error("k4a_device_get_serialnum");
				}

				// シリアルナンバーを取得する
				std::vector<char> serial(serial_number_size);
				ret = k4a_device_get_serialnum(handle_.get(), &serial[0], &serial_number_size);
				if (ret != K4A_BUFFER_RESULT_SUCCEEDED) {
					throw std::runtime_error("k4a_device_get_serialnum");
				}

				serial_ = &serial[0];
			}
		}

		bool isOpen()
		{
			return handle_.get() != 0;
		}

		void startCamera(k4a_device_configuration_t *config)
		{
			if (!isOpen()) {
				throw std::runtime_error("Device not opened.");
			}

			if (isCameraRunning) {
				stopCamera();
			}

			k4a_device_start_cameras(handle_.get(), config);
			config_ = *config;
			isCameraRunning = true;
		}

		void stopCamera()
		{
			if (!isOpen()) {
				throw std::runtime_error("Device not opened.");
			}

			if (!isCameraRunning) {
				throw std::runtime_error("Camera not running.");
			}

			k4a_device_stop_cameras(handle_.get());
			isCameraRunning = false;
		}

		// IMUはカメラが動作している状態であること
		void startImu()
		{
			if (!isOpen()) {
				throw std::runtime_error("Device not opened.");
			}

			if (!isCameraRunning) {
				throw std::runtime_error("Camera not running.");
			}

			if (isImuRunning) {
				stopImu();
			}

			k4a_device_start_imu(handle_.get());
			isImuRunning = true;
		}

		void stopImu()
		{
			if (!isOpen()) {
				throw std::runtime_error("Device not opened.");
			}

			if (!isImuRunning) {
				throw std::runtime_error("Imu not running.");
			}

			k4a_device_stop_imu(handle_.get());
			isImuRunning = false;
		}

		const std::string& gtSerialNumber() const
		{
			return serial_;
		}

		static uint32_t deviceCouunt()
		{
			return k4a_device_get_installed_count();
		}

		SyncJack getJackState()
		{
			bool sync_in_jack_connected = false;
			bool sync_out_jack_connected = false;
			k4a_device_get_sync_jack(handle_.get(), &sync_in_jack_connected, &sync_out_jack_connected);

			return SyncJack(sync_in_jack_connected, sync_out_jack_connected);
		}

		k4a_imu_sample_t getImuSample()
		{
			k4a_imu_sample_t sample;
			k4a_device_get_imu_sample(handle_.get(), &sample, K4A_WAIT_INFINITE);
			return sample;
		}

	private:

		std::shared_ptr<_k4a_device_t> handle_;
		std::string serial_;
		k4a_device_configuration_t config_;

		bool isCameraRunning = false;
		bool isImuRunning = false;
	};

	// 画像データ
	class Image
	{
		friend Transformation;

	public:

		Image(k4a_image_t image)
			: image_(std::shared_ptr<_k4a_image_t>(image, k4a_image_release))
		{
		}

		uint8_t* getBuffer()
		{
			return k4a_image_get_buffer(image_.get());
		}

		size_t getSize() const
		{
			return k4a_image_get_size(image_.get());
		}

		k4a_image_format_t getFormat() const
		{
			return k4a_image_get_format(image_.get());
		}

		int getWidth() const
		{
			return k4a_image_get_width_pixels(image_.get());
		}

		int getHeight() const
		{
			return k4a_image_get_height_pixels(image_.get());
		}

		int getStride() const
		{
			return k4a_image_get_stride_bytes(image_.get());
		}

		uint64_t getTimestamp() const
		{
			return k4a_image_get_timestamp_usec(image_.get());
		}

	private:

		std::shared_ptr<_k4a_image_t> image_ = 0;
	};

	// フレームデータ
	class Capture
	{
	public:

		Capture()
		{
		}

		Capture(Sensor& kinect)
		{
			auto ret = Open(kinect, K4A_WAIT_INFINITE);
			if (!ret) {
				throw std::runtime_error("k4a_device_get_capture K4A_WAIT_RESULT_TIMEOUT");
			}
		}

		bool Open(Sensor& kinect, int32_t timeout_in_ms = 0)
		{
			k4a_capture_t capture = 0;
			auto ret = k4a_device_get_capture(kinect.handle_.get(), &capture, timeout_in_ms);
			if (ret == K4A_WAIT_RESULT_FAILED) {
				throw std::runtime_error("k4a_device_get_capture");
			}
			else if (ret == K4A_WAIT_RESULT_TIMEOUT) {
				return false;
			}

			capture_ = std::shared_ptr<_k4a_capture_t>(capture, k4a_capture_release);

			return true;
		}

		Image getColorImage()
		{
			return Image(k4a_capture_get_color_image(capture_.get()));
		}

		Image getDepthImage()
		{
			return Image(k4a_capture_get_depth_image(capture_.get()));
		}

		Image getIrImage()
		{
			return Image(k4a_capture_get_ir_image(capture_.get()));
		}

	private:

		std::shared_ptr<_k4a_capture_t> capture_;
	};

	class Transformation
	{
	public:

		void Open(Sensor& sensor)
		{
			// キャリブレーション情報を取得する
			auto ret = k4a_device_get_calibration(sensor.handle_.get(), sensor.config_.depth_mode, sensor.config_.color_resolution, &calibration_);
			if (ret != K4A_RESULT_SUCCEEDED) {
				throw std::runtime_error("k4a_device_get_calibration");
			}

			// 変換用ハンドルを取得する
			transformation_ = std::shared_ptr<_k4a_transformation_t>(k4a_transformation_create(&calibration_), k4a_transformation_destroy);
		}

		Image ColorToDepth(const Image& color, const Image& depth)
		{
			k4a_image_t transformed_color_image = 0;
			k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, depth.getWidth(), depth.getHeight(), depth.getWidth() * 4, &transformed_color_image);

			auto ret = k4a_transformation_color_image_to_depth_camera(transformation_.get(), depth.image_.get(), color.image_.get(), transformed_color_image);
			if (ret != K4A_RESULT_SUCCEEDED) {
				throw std::runtime_error("k4a_transformation_color_image_to_depth_camera");
			}

			return Image(transformed_color_image);
		}

		Image DepthToPointCloud(const Image& depth)
		{
			k4a_image_t xyz_image = 0;
			k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, depth.getWidth(), depth.getHeight(), depth.getWidth() * 6, &xyz_image);

			auto ret = k4a_transformation_depth_image_to_point_cloud(transformation_.get(), depth.image_.get(), K4A_CALIBRATION_TYPE_DEPTH, xyz_image);
			if (ret != K4A_RESULT_SUCCEEDED) {
				throw std::runtime_error("k4a_transformation_depth_image_to_point_cloud");
			}

			return Image(xyz_image);
		}

	private:
		
		k4a_calibration_t calibration_ = { 0 };
		std::shared_ptr<_k4a_transformation_t> transformation_;
	};
}

