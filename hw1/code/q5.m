function [R,t] = q5(P, Q)
    if nargin ~= 2
	    error("Missing parameters");
    end

    % get centroid of the points
    centroid_P = mean(P);
    centroid_Q = mean(Q);

    N = size(P,1);
    % calculate the matrix AB^T and take its SVD
    H = (P - repmat(centroid_P, N, 1))' * (Q - repmat(centroid_Q, N, 1));
    [U,~,V] = svd(H);

    R = V*U';

    if det(R) < 0
        V(:,3) = -1*V(:,3);
        R = V*U';
    end

    % get the translation from the rotation and centroids
    t = centroid_Q' - R*centroid_P';
end