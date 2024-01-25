import { Component, inject } from '@angular/core';
import { RouterOutlet } from '@angular/router';

@Component({
  selector: 'app-root',
  standalone: true,
  imports: [RouterOutlet],
  templateUrl: './app.component.html',
  styleUrl: './app.component.scss'
})
export class AppComponent {
  title = 'WebUI';
 
  //firestore = inject(Firestore);

  ngOnInit() {
   // getDocs(collection(this.firestore, "testPath")).then((response) => {
   //   console.log(response.docs)
   // }
}
