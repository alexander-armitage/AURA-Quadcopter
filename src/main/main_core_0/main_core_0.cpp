#include "main_core_0.h"

#include "Madgwick.h"

namespace main {

void core_0_task(void* args) {
  uint64_t loop_counter = 0;

  auto controlOutputQueue = static_cast<QueueHandle_t>(args);

  sensor_data my_data{};

  // I2C Bus definition
  i2c::i2c_bus my_bus(SDA, SCL);

  // I2C Device definitions
  bmp280::bmp280 my_bmp(&my_bus);
  icm20948::icm20948 my_icm(&my_bus);
  ina219::ina219 my_ina(&my_bus);
  pca9685::pca9685 my_pca(&my_bus);

  // PID Controller defintions
  pid::pid pid_x(0.65F, 0.55F, 0.0001F, 0.0F, 25.0F);
  pid::pid pid_y(0.65F, 0.55F, 0.0001F, 0.0F, 25.0F);
  pid::pid pid_z(0.9F, 0.75F, 0.0001F, 0.0F, 25.0F);

  // ESC object definitions
  pca9685::pca_esc esc0(&my_pca, 0);
  pca9685::pca_esc esc1(&my_pca, 1);
  pca9685::pca_esc esc2(&my_pca, 2);
  pca9685::pca_esc esc3(&my_pca, 3);

  // FS-A8S Receiver definition
  fsa8s::receiver my_receiver(RC_RX);

  // Orientation definitions
  ori::Quat ori_setpoint_quat{};

  ori::Vect gyro{};
  ori::Vect ori_setpoint{};
  ori::Vect ori_euler{};
  ori::Vect ori_error{};

  ori::Vect meas_mag{};
  ori::Vect meas_acc{};
  ori::Vect mag_exp{};
  ori::Vect acc_exp(0.0F, 0.0F, 1.0F);

  ori::Quat ori_quat{};
  ori::Quat meas_mag_q{};
  ori::Quat mag_exp_q{};

  // Time handling definitons
  float dt{};
  int64_t last_time = esp_timer_get_time();
  int64_t delta_us{};
  int64_t now{};

  // Control input definitions
  float yaw = 0.0F;
  uint8_t throttle{};

  // Calibration
  vTaskDelay(pdMS_TO_TICKS(3000));
  my_icm.calibrate_gyro(1000);

  // Update IMU sensors
  my_icm.update();
  my_icm.update_mag();

  meas_acc =
      ori::Vect(my_icm.acc_x(), my_icm.acc_y(), my_icm.acc_z()).normalise();
  meas_mag =
      ori::Vect(my_icm.mag_x(), my_icm.mag_y(), my_icm.mag_z()).normalise();

  // Compute the expected magnetometer values
  ori_quat = ori::Vect::quat_from_two_vect(meas_acc, acc_exp);
  meas_mag_q = ori::Quat(0.0F, meas_mag.x(), meas_mag.y(), meas_mag.z());
  mag_exp_q = ori_quat.conjugate() * meas_mag_q * ori_quat;

  mag_exp = ori::Vect(mag_exp_q.i(), mag_exp_q.j(), mag_exp_q.k());

  // Madgwick filter
  ori::madgwick filter(0.05, mag_exp, acc_exp);

  /*float minX = std::numeric_limits<float>::max();
  float minY = std::numeric_limits<float>::max();
  float minZ = std::numeric_limits<float>::max();
  float maxX = -std::numeric_limits<float>::max();
  float maxY = -std::numeric_limits<float>::max();
  float maxZ = -std::numeric_limits<float>::max();

  while (true) {
    float x, y, z, avg_rad;
    float radX, radY, radZ;
    float scaleX, scaleY, scaleZ;

    my_icm.update_mag();
    x = my_icm.mag_x();
    y = my_icm.mag_y();
    z = my_icm.mag_z();

    // Update min/max
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
    if (z < minZ) minZ = z;
    if (z > maxZ) maxZ = z;

    radX = (maxX - minX) / 2;
    radY = (maxY - minY) / 2;
    radZ = (maxZ - minZ) / 2;

    avg_rad = (radX + radY + radZ) / 3.0;
    scaleX = avg_rad / radX;
    scaleY = avg_rad / radY;
    scaleZ = avg_rad / radZ;

    avg_rad =
        printf("%f %f %f %f %f %f\n", (minX + maxX) / 2.0F, scaleX,
               (minY + maxY) / 2.0F, scaleZ, (minZ + maxZ) / 2.0F, scaleZ);
  }*/

  // Main loop
  while (true) {
    loop_counter++;

    // Calculate delta time
    now = esp_timer_get_time();
    delta_us = now - last_time;
    last_time = now;
    dt = static_cast<float>(delta_us) / 1e6F;

    // Update necessary sensors
    my_icm.update();
    my_icm.update_mag();
    my_receiver.update();

    // Compute control inputs and desired state
    my_data.ch1 = static_cast<float>(my_receiver.ch1() - TRANSMITTER_ZERO) *
                  INPUT_TO_MAX_ANGLE;
    my_data.ch2 = static_cast<float>(my_receiver.ch2() - TRANSMITTER_ZERO) *
                  INPUT_TO_MAX_ANGLE;
    my_data.ch3 = static_cast<float>(my_receiver.ch3());
    my_data.ch4 = static_cast<float>(my_receiver.ch4() - TRANSMITTER_ZERO) *
                  INPUT_TO_MAX_ANGLE;
    my_data.ch5 = static_cast<float>(my_receiver.ch5());
    my_data.ch6 = static_cast<float>(my_receiver.ch6());

    throttle =
        (static_cast<float>(my_data.ch3 - TRANSMITTER_MIN) * INPUT_TO_PRECENT) +
        ARMED_THROTTLE;

    yaw += my_data.ch4 * dt;
    if (yaw > PI) {
      yaw = -PI + (yaw - PI);
    }
    if (yaw < -PI) {
      yaw = PI + (yaw + PI);
    }

    // Calculate current orientation
    ori::Vect gyro(my_icm.gyro_x(), my_icm.gyro_y(), my_icm.gyro_z());
    ori::Vect meas_acc(my_icm.acc_x(), my_icm.acc_y(), my_icm.acc_z());
    ori::Vect meas_mag(my_icm.mag_x(), my_icm.mag_y(), my_icm.mag_z());

    float acc_mag = meas_acc.mag();
    meas_acc = meas_acc.normalise();
    meas_mag = meas_mag.normalise();

    // Use filter if accelerometer is roughly gravity
    if (acc_mag > 9.0F && acc_mag < 10.6F) {
      ori_quat = filter.update_acc_gyro(ori_quat, gyro, meas_acc, dt);
    } else {
      ori_quat = ori_quat.update(gyro, dt);
    }

    // Calculate error direction
    ori_error = ori_quat.calc_error(ori::Quat(1.0F, 0.0F, 0.0F, 0.0F));

    // Check if armed, if not armed, continue
    if (my_data.ch5 <= 1500) {
      pid_x.reset();
      pid_y.reset();
      pid_z.reset();

      ESP_ERROR_CHECK(esc0.set_throttle(0));
      ESP_ERROR_CHECK(esc1.set_throttle(0));
      ESP_ERROR_CHECK(esc2.set_throttle(0));
      ESP_ERROR_CHECK(esc3.set_throttle(0));

      continue;
    }

    // Avoid integral windup by only enabling when flying
    if (my_data.ch3 < 1050) {
      pid_x.enable_disable_interal(false);
      pid_y.enable_disable_interal(false);
      pid_z.enable_disable_interal(false);
    } else {
      pid_x.enable_disable_interal(true);
      pid_y.enable_disable_interal(true);
      pid_z.enable_disable_interal(true);
    }

    // Check if stabalised or acro mode
    if (my_data.ch6 < 1250) {
      pid_x.set_setpoint((my_data.ch1) * 10);
      pid_y.set_setpoint((my_data.ch2) * 10);
      pid_z.set_setpoint((my_data.ch4) * 10);
    } else {
      pid_x.set_setpoint((my_data.ch1 - ori_error.x()) * 5);
      pid_y.set_setpoint((my_data.ch2 - ori_error.y()) * 5);
      pid_z.set_setpoint((yaw - ori_error.z()) * 5);
    }

    //  Calculate control outputs
    my_data.pid_x = pid_x.update(gyro.x(), dt);
    my_data.pid_y = pid_y.update(gyro.y(), dt);
    my_data.pid_z = pid_z.update(gyro.z(), dt);

    // Update ESC values (throttle %)
    my_data.esc0 = throttle - my_data.pid_x - my_data.pid_y + my_data.pid_z;
    my_data.esc1 = throttle + my_data.pid_x - my_data.pid_y - my_data.pid_z;
    my_data.esc2 = throttle + my_data.pid_x + my_data.pid_y + my_data.pid_z;
    my_data.esc3 = throttle - my_data.pid_x + my_data.pid_y - my_data.pid_z;

    ESP_ERROR_CHECK(esc0.set_throttle(my_data.esc0));
    ESP_ERROR_CHECK(esc1.set_throttle(my_data.esc1));
    ESP_ERROR_CHECK(esc2.set_throttle(my_data.esc2));
    ESP_ERROR_CHECK(esc3.set_throttle(my_data.esc3));

    // Data log every 50 loops to keep control loop fast
    if (loop_counter % 50 != 0) {
      continue;
    }

    my_bmp.update();

    ori_euler = ori_quat.to_euler();
    ori_euler = ori_euler.to_degrees();

    my_data.ori_x = ori_euler.x();
    my_data.ori_y = ori_euler.y();
    my_data.ori_z = ori_euler.z();

    my_data.set_x = (my_data.ch1 - ori_error.x()) * RAD_TO_DEG;
    my_data.set_y = (my_data.ch2 - ori_error.y()) * RAD_TO_DEG;
    my_data.set_z = (yaw - ori_error.z()) * RAD_TO_DEG;

    my_data.gyro_x = gyro.x();
    my_data.gyro_y = gyro.y();
    my_data.gyro_z = gyro.z();

    my_data.dt = dt;

    my_data.temp = my_bmp.get_temperature();
    my_data.height = my_bmp.get_altitude();

    my_data.voltage = my_ina.get_voltage();
    my_data.current = my_ina.get_current();
    my_data.power = my_ina.get_power();

    xQueueOverwrite(controlOutputQueue, &my_data);
  }
}

}  // namespace main
