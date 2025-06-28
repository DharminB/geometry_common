#!groovy
pipeline {
    agent {
        dockerfile {
            filename 'Dockerfile'
            additionalBuildArgs '--no-cache'
            args '-u root --entrypoint=""'
        }
    }
    stages {
        stage('Build') {
            steps {
                sh '''
                    cd /workspace/ros2_ws
                    /ros_entrypoint.sh colcon build --event-handlers desktop_notification- status-
                '''
            }
        }
        stage('Test') {
            steps {
                sh '''
                    . /workspace/ros2_ws/install/setup.sh
                    colcon test --packages-up-to geometry_common --event-handlers console_direct+
                '''
                sh '''
                    mkdir -p test_results
                    cp /workspace/ros2_ws/build/geometry_common/test_results/geometry_common/geometry_common_test.gtest.xml ./test_results/
                '''
            }
        }
    }
    post {
        always {
            archiveArtifacts artifacts: 'test_results/*.xml', fingerprint: true
            xunit (
                thresholds: [ skipped(failureThreshold: '0'), failed(failureThreshold: '0') ],
                tools: [ GoogleTest(pattern: 'test_results/*.xml') ]
            )
        }
    }
}
