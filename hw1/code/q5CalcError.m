function rmse = q5CalcError(ret_R, ret_t, P, Q, n)

reconstructQ = (ret_R*P') + repmat(ret_t, 1, n);
reconstructQ = reconstructQ';

% Find the error
error = reconstructQ - Q;
error = error .* error;
error = sum(error(:));
rmse = sqrt(error/n);

end