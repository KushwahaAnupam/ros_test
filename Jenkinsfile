pipeline {
    agent any

    environment {
        ROS_DISTRO = 'humble'
        WORKSPACE = "${env.WORKSPACE}"
        PACKAGE = 'hello_publisher'  // Replace with your actual ROS2 package name
        ROS_LOCALHOST_ONLY = 0
        ROS_DOMAIN_ID = 10
    }

    stages {
        stage('Checkout') {
            steps {
                echo 'Checking out source code...'
                checkout scm
            }
        }

        stage('Build') {
            steps {
                echo 'Building ROS2 workspace with colcon...'
                sh '''
                  bash -c "
                    source /opt/ros/${ROS_DISTRO}/setup.bash &&
                    colcon build --packages-select ${PACKAGE} --symlink-install
                  "
                '''
            }
        }


        stage('Run Publisher') {
            steps {
                // timeout(time: 10, unit: 'SECONDS') {
                  echo 'Running the ROS2 publisher node...'
                  sh '''
                    bash -c "
                      source /opt/ros/${ROS_DISTRO}/setup.bash &&
                      source install/setup.bash &&
                      ros2 run ${PACKAGE} hello_node
                    "
                  '''
                // }
            }
        }

        // stage('Run Tests') {
        //     steps {
        //         echo 'Running tests with colcon test...'
        //         sh '''
        //           bash -c "
        //             source /opt/ros/${ROS_DISTRO}/setup.bash &&
        //             source install/setup.bash &&
        //             colcon test --packages-select ${PACKAGE} &&
        //             colcon test-result --verbose
        //           "
        //         '''
        //     }
        // }

        // stage('Archive Test Reports') {
        //     steps {
        //         echo 'Archiving test results...'
        //         junit '**/build/test_results/**/*.xml'
        //     }
        // }
    }

    // post {
    //     always {
    //         echo 'Cleaning workspace...'
    //         cleanWs()
    //     }
    //     success {
    //         echo 'Build and tests passed!'
    //     }
    //     failure {
    //         echo 'Build or tests failed!'
    //     }
    // }
}
