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
                    cd /workspace/catkin_ws
                    /ros_entrypoint.sh catkin build --no-status
                '''
            }
        }
        stage('Test') {
            steps {
                sh '''
                    echo "Sourcing /workspace/catkin_ws/devel/setup.sh silently"
                    set +x
                    . /workspace/catkin_ws/devel/setup.sh
                    set -x
                    cd /workspace/catkin_ws/src/geometry_common
                    catkin test --this --no-status
                '''
                sh '''
                    mkdir -p test_results
                    cp /workspace/catkin_ws/build/geometry_common/test_results/geometry_common/gtest-geometry_common_test.xml ./test_results/
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
