# Manas_tasks
To store taskphase related files. 
---------------------------------------------------
Task #2.0 :   
Using Linear Regression for Closing Price Prediction of stock market.       
Dataset consisted of 16 features and 283,085 training examples.   

Steps taken:
1. Normalization of all features [feature-mean/standard deviation]   
2. Multiple Linear Regression (using vectors) [f_w,b(x)=w.x + b]  
3. Using gradient descent to minimize Mean squared error cost function [J = (1/2m)*summation(f_wb - y)^2]  
4. Using R-squared, Adjusted R-squared, mean square value and root mean square value to find most efficient alpha, iterations.

Final results for training :   
Cost=0.004642 | R^2=0.9908 | Adjusted R^2=0.9908 | Mean squared error=0.0092 | Root Mean squared error=0.095  
Final results for testing :   
Cost=0.020663 | R^2=0.9587 | Adjusted R^2=0.9587 | Mean squared error=0.0413 | Root Mean squared error=0.2033
________________________________________________________________________________
* Since RMSE and MSE are much greater for testing data than training data,model might be overfitted.
* Since there is minimal difference in R^2 and adjusted R^2, therefore there is no issue with the features. Also, while the difference in MSE is high, both continue to go down with increase in epochs and learning rate, so difference might be amplified since values are already pretty small.
* Despite this, predictions still show very high accuracy for both training and testing data as shown by the high R^2 in both cases.

Final alpha = 0.37 and iterations = 100 for best result in least computation time.
---------------------------------------------------
Note : linear_regression2.ipynb contains same task, but if we had used Normal Equation instead of Gradient Descent. It is only a benchmark for expected performance from main model
________________________________________________________________________________