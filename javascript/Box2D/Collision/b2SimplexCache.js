var b2SimplexCache = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2SimplexCache.prototype.__constructor = function(){}
b2SimplexCache.prototype.__varz = function(){
this.indexA =  new Vector(3);
this.indexB =  new Vector(3);
}
// static attributes
// static methods
// attributes
b2SimplexCache.prototype.metric =  null;
b2SimplexCache.prototype.count =  0;
b2SimplexCache.prototype.indexA =  new Vector(3);
b2SimplexCache.prototype.indexB =  new Vector(3);
// methods