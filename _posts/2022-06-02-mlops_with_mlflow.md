---
title:  "MLOps with MLFlow"
author: matheus
date:   2022-06-02 10:00:00 -0300
categories: [DevOps, MLOps]
tags: [machine learning, devops]
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

{% include image.html url="/mlops_with_mlflow/mlops_components.png" description="Extracted from: <a href='https://databricks.com/glossary/mlops'>https://databricks.com/glossary/mlops</a>" %}


The components of 

# Maturity levels of MLOps processes

## Manual

## ML Pipeline Automation

## ML Pipeline Automation with CI/CD

# Using MLFlow

## Services Necessary

### Setting up Minio

### Setting up MLFlow

## Training a model

### Dataset used

### Model trained

### Managing Experiments on MLFlow

### Deploying a model

{% include youtube.html url="https://www.youtube.com/embed/G7300HZEiOo" %}

{% include signature.html %}

