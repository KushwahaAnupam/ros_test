pipeline {
    agent any

    environment {
        // Source your ROS2 install and workspace overlays
        ROS_DISTRO = 'humble'
        WORKSPACE = "${env.WORKSPACE}"
    }

    stages {
        stage('Checkout') {
            steps {
                echo 'Checking out source code...'
                checkout scm
            }
        }

        stage('Setup ROS2 Environment') {
            steps {
                echo 'Setting up ROS2 environment...'
                // Source ROS2 Humble and workspace setup.bash
                sh '''
                  source /opt/ros/${ROS_DISTRO}/setup.bash
                  # You can source overlay if you have one:
                  # source ${WORKSPACE}/install/setup.bash || true
                '''
            }
        }

        stage('Install Dependencies') {
            steps {
                echo 'Installing Python dependencies...'
                sh '''
                  python3 -m pip install -r requirements.txt || true
                '''
            }
        }

        stage('Build') {
            steps {
                echo 'Building ROS2 workspace with colcon...'
                sh '''
                  source /opt/ros/${ROS_DISTRO}/setup.bash
                  colcon build --packages-select your_package_name --symlink-install
                '''
            }
        }

        stage('Run Tests') {
            steps {
                echo 'Running tests with colcon test...'
                sh '''
                  source /opt/ros/${ROS_DISTRO}/setup.bash
                  source install/setup.bash
                  colcon test --packages-select your_package_name
                  colcon test-result --verbose
                '''
            }
        }

        stage('Archive Test Reports') {
            steps {
                echo 'Archiving test results...'
                junit '**/build/test_results/**/*.xml'
            }
        }
    }

    post {
        always {
            echo 'Cleaning workspace...'
            cleanWs()
        }
        success {
            echo 'Build and tests passed!'
        }
        failure {
            echo 'Build or tests failed!'
        }
    }
}
