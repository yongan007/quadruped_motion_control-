function [dist,diff_dist]=get_distance(FK_fun,q,xb)

diff_dist = FK_fun(q)-xb;

dist = norm(diff_dist);
end
