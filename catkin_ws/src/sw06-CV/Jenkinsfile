pipeline {
  agent any
  stages {
    stage('Prepare') {
      steps {
        sh 'pip3 install --upgrade duckietown-shell'
        sh 'dts update'
    	  sh 'dts install devel'
      }
    }
    stage('Pre-Clean') {
      steps {
        sh 'dts devel clean'
      }
    }
    stage('Build') {
      steps {
        sh 'dts devel build --no-multiarch'
      }
    }
    stage('Push') {
      steps {
        withDockerRegistry(credentialsId: 'DockerHub', url: 'https://index.docker.io/v1/') {
          sh 'dts devel push'
        }
      }
    }
    stage('Post-Clean') {
      steps {
        sh 'dts devel clean'
      }
    }
  }
  post {
      always {
          sh 'dts devel clean'
          cleanWs()
      }
  }
}
