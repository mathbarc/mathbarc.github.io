---
title:  "MLOps with MLFlow"
author: matheus
date:   2023-01-02 10:00:00 -0300
categories: [DevOps, MLOps]
tags: [machine learning, docker]
pin: false
---

<div>
<img src="{{site.baseurl}}/github.png" width=20px height=20px /> <a href="https://github.com/mathbarc/mlops_with_mlflow">mathbarc/mlops_with_mlflow</a>
</div>


# What is MLOps?

![MLOps Model]({{site.baseurl}}/mlops_with_mlflow/mlops.png "MLOps Model")

MLOps is a set of practices used to reduce the distance between the creation of Machine Learning models and the final user of this model. This concept is based on the DevOps model but with additional steps for Machine Learning development:

- Capture and organization of data, the most important intake of machine learning approachs;
- Selection and training of models compatible with patterns which extraction from dataset is desired;
- Validation of models with data different then used in training.

These three steps are included as a extension of the development fase but generally on projects those steps occur in parellel, since the time necessary for maturing a machine learning model and the code base are different.

# Different components of MLOps process

{% include image.html url="/mlops_with_mlflow/mlops_components.png" description="Extracted from: <a href='https://databricks.com/glossary/mlops'>databricks.com</a>" %}


With aim of reducing disparities between code and ML models, the following steps are adopted into the development phase:

- **Exploratory data analysis:** In this step data scientists analyse the dataset to comprehand the problem and acess the viability of the solution;
- **Data preparation:** At this point the data is cleaned, organized and annotated;
- **Model training:** The organized dataset is divided into a training set and test set. Different models are created and trained on the data to extract its patterns and aftwards the test set is used to assure generalization of the models;
- **Model serving:** When a model reaches the quality standard desired it is moved to production where it will be used to infer new data;
- **Monitoring:** To garantee that the model keeps making decisions on the same quality desired on the training step, it is necessary to log results data and setup a feedback process from the users of this model. New data that performs badly is separated to integrate the dataset for new model training iterations.

# Maturity levels of MLOps processes

## Manual

{% include image.html url="/mlops_with_mlflow/manual_process.png" description="Extracted from: <a href='https://cloud.google.com/architecture/mlops-continuous-delivery-and-automation-pipelines-in-machine-learning
'>cloud.google.com</a>" %}

The manual process consists in creating each step of a Machine Learning Pipeline through scripts or notebooks that are manually started after each step of the pipeline. This generally is the first process implemented at the start of a project or in its proof of concept phase due to the lower difficulty and cost of implementation. 

The main flaw of this process is the cost of mainteinance due to the difficulty of migrating the pipeline to different environment and the difficulty to update the pipeline with new features, this flaw fosters a disparity between the code that goes to production and the models been trained for these software modules.

## ML Pipeline Automation

{% include image.html url="/mlops_with_mlflow/automated_ml_pipeline.png" description="Extracted from: <a href='https://cloud.google.com/architecture/mlops-continuous-delivery-and-automation-pipelines-in-machine-learning'>cloud.google.com</a>" %}

This level is generally a progression from the Manual Process when the project reaches a initial user base and the manual process begins to be costly to maintain. This automatization allows for faster execution of experiments and increases simmetry between the code that are used in all environments of the solution, which allows replication of the production environment in the developer's environment. 

This possibility brings a opportunity: Continuous delivery of models. New improvements and models can be created, validated and delivered faster and cheaper. 

## ML Pipeline Automation with CI/CD

{% include image.html url="/mlops_with_mlflow/automated_cicd_pipeline.png" description="Extracted from: <a href='https://cloud.google.com/architecture/mlops-continuous-delivery-and-automation-pipelines-in-machine-learning'>cloud.google.com</a>" %}

On this stage there is a automation of tests and packaging of software and models using CI/CD pipelines. In this format both the models and code are tested together after every alteration by the CI workflow and then, if no problems are detected, packaged and delivered to next environments through a CD workflow. This format allows for a increased speed of delivery of new features and fixes and also better quality control. 

Pipelines of this level are generally used by big teams on projects already with a considerable user base and with higher complexibility.

# Using MLFlow

On this article we will setup the MLFlow service. MLFlow is a open source MLOps tool which allows trackking the evolution of models and also their versioning, allowing control over the model's cicle of life. After setting up MLFlow, it will be shown how to train and serve a model using this tool.

## Setting up the environment

A simple way to configure all the tools needed is to use [Docker](https://docs.docker.com/get-docker/). The following docker-compose.yml file contains the configuration of 3 services: MinIO, Postgres and MLFlow. MinIO is a S3 bucket service that can be used locally to store files as we would on AWS S3. Postgres is a relational database used by MLFlow to store metadata about the models. MLFlow is the service we will comunicate to log metrics and control the model versioning.

```yaml
version: "3.7"

services:
    minio:
      image: minio/minio
      environment:
        - MINIO_CONSOLE_ADDRESS=:9001
        - MINIO_ROOT_USER=mlops-demo
        - MINIO_ROOT_PASSWORD=mlops-demo
      ports:
        - "9000:9000"
        - "9001:9001"
      command:
        - server
        - /data
      volumes:
        - minio-data:/data

    postgres:
      image: postgres
      environment:
        - POSTGRES_USER=mlops-db
        - POSTGRES_PASSWORD=mlops-db
        - POSTGRES_DB=mlops-db
        - PGDATA=/var/lib/postgresql/data
      ports:
        - "5432:5432"
      volumes:
        - mlflow-db-data:/var/lib/postgresql/data
    
    mlflow:
      image: ghcr.io/mathbarc/mlflow:master
      environment:
        - MLFLOW_S3_ENDPOINT_URL=http://minio:9000/
        - AWS_ACCESS_KEY_ID=mlops-demo
        - AWS_SECRET_ACCESS_KEY=mlops-demo
      command: ["server","--host", "0.0.0.0", "--port", "9002", "--backend-store-uri","postgresql://mlops-db:mlops-db@postgres:5432/mlops-db", "--default-artifact-root","s3://mlops"]
      ports:
        - "9002:9002"
      depends_on:
        - minio
        - postgres

volumes:
  minio-data: 
  mlflow-db-data:

```

After saving the file above to disk with the name docker-compose.yml we can run the following command to setup our MLOps environment.

```bash
docker-composer up -d
```

## Training a model



### Dataset used

### Model trained

### Managing Experiments on MLFlow

### Deploying a model

{% include youtube.html url="https://www.youtube.com/embed/G7300HZEiOo" %}

{% include signature.html %}

