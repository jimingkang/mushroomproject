import { HttpClient } from '@angular/common/http';
import { Injectable } from '@angular/core';

import { Observable } from 'rxjs';

@Injectable({
  providedIn: 'root'
  
})
export class ProductService {

  constructor(private http: HttpClient) { }

  getCategory() {
    return this.http.get(Constant.API_END_POINT + Constant.METHODS.GET_ALL_CATEGORY);
  }

  getProductsByCategory(id: number) {
    return this.http.get(Constant.API_END_POINT + Constant.METHODS.GET_ALL_PRODUCT_BY_CATEGORY +  id);
  }
  getProducts():Observable<any> {
    return this.http.get(Constant.API_END_POINT + Constant.METHODS.GET_ALL_PRODUCT);
  }
  saveProduct(obj: any) {
    return this.http.post(Constant.API_END_POINT + Constant.METHODS.CREATE_PRODUCT, obj);
  }
  updateProduct(obj: any) {
    return this.http.post(Constant.API_END_POINT + Constant.METHODS.UPDATE_PRODUCT, obj);
  }

  deleteProduct(id: any) {
    return this.http.get(Constant.API_END_POINT + Constant.METHODS.DELETE_PRODUCT + id);
  }
}

export const Constant = {
  API_END_POINT:'https://freeapi.miniprojectideas.com/api/BigBasket/',
  METHODS: {
      GET_ALL_PRODUCT: 'GetAllProducts',
      GET_ALL_CATEGORY: 'GetAllCategory',
      GET_ALL_PRODUCT_BY_CATEGORY: 'GetAllProductsByCategoryId?id=',

      CREATE_PRODUCT: 'CreateProduct',
      UPDATE_PRODUCT: 'UpdateProduct',
      DELETE_PRODUCT: 'DeleteProductById?id='
  }
}