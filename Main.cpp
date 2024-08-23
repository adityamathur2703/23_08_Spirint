#include<iostream>
#include<vector>
#include<cmath>
#include<gtest/gtest.h>

std::vector<float> kalmanFilter(const std::vector<float> &gpsSignals);

TEST(KalmanFilterTest, HandlesBasicInput)
{
    // Define the input GPS signals
    std::vector<float> gpsSignals = {123.456, 122.789, 121.234, 120.567, 119.890, 119.123, 118.456, 117.890};

    // Call the Kalman filter function
    std::vector<float> result = kalmanFilter(gpsSignals);
    std::vector<float> expected = {123.456, 122.837, 121.347, 120.622, 119.942, 119.182, 118.508, 117.935};

    // Expected result (adjust these values based on expected filter output)
    // Ensure the result matches the expected values
    ASSERT_EQ(result.size(), expected.size()); // Ensure vectors have the same size
    for (size_t i = 0; i < expected.size(); ++i)
    {
        EXPECT_NEAR(result[i], expected[i], 1e-3); // Compare each element with a tolerance for floating point comparison
    }
}
// Function to implement the Kalman filter for velocity calculation
std::vector<float> kalmanFilter(const std::vector<float> &gpsSignals)
{
    // Initialize variables
    float estimate = gpsSignals[0]; // initial estimate
    float error_estimate = 1.0f;    // initial estimate error
    float error_measure = 1.0f;     // measurement error
    float process_noise = 0.1f;     // process noise

    std::vector<float> velocity(gpsSignals.size());

    // Kalman filter process
    for (size_t i = 0; i < gpsSignals.size(); ++i)
    {
        float kalman_gain;
        // Measurement update
        kalman_gain = error_estimate / (error_estimate + error_measure);
        estimate += kalman_gain * (gpsSignals[i] - estimate);
        error_estimate *= 1.0f - kalman_gain;
        error_estimate += estimate * process_noise;

        // Save the estimated velocity
        velocity[i] = estimate;
    }

    return velocity;
}

int main(void)
{
    std::vector<float> gpsSignals = {123.456, 122.789, 121.234, 120.567, 119.890, 119.123, 118.456, 117.890};

    // Calculate the vehicle's velocity using the Kalman filter
    std::vector<float> velocity = kalmanFilter(gpsSignals);

    // Output the filtered velocity
    for (size_t i = 0; i < velocity.size(); ++i)
    {
        std::cout << "Velocity[" << i << "] = " << velocity[i] << std::endl;
    }
    testing::InitGoogleTest();
    return RUN_ALL_TESTS();
    // return 0;
}
